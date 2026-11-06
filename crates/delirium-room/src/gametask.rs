//! RDR2's tasks fire their Euphoria messages from sets in the player's own physicstasks file (data_0.rpf/data/tune,
//! exported as XML). `rdr2 task <set>` reads that file at run time from `--tune-dir <folder>` (or RDR2_TUNE_DIR) and
//! sends the set's messages through the room's usual message line. Nothing of the game's is in this source: names in
//! the file are joaat hashes, matched here against the port's own message and parameter names.

use delirium::msg::{Kind, Message};

/// joaat of the lower case, the hash the game's files use.
pub fn joaat(s: &str) -> u32 {
    let mut h = 0u32;
    for c in s.bytes() {
        h = h.wrapping_add(c.to_ascii_lowercase() as u32);
        h = h.wrapping_add(h << 10);
        h ^= h >> 6;
    }
    h = h.wrapping_add(h << 3);
    h ^= h >> 11;
    h.wrapping_add(h << 15)
}

#[derive(Default, Debug)]
pub struct Node {
    pub tag: String,
    pub attrs: Vec<(String, String)>,
    pub text: String,
    pub kids: Vec<Node>,
}

impl Node {
    fn kid(&self, tag: &str) -> Option<&Node> {
        self.kids.iter().find(|k| k.tag == tag)
    }
    fn attr(&self, a: &str) -> Option<&str> {
        self.attrs.iter().find(|x| x.0 == a).map(|x| x.1.as_str())
    }
    fn value(&self) -> Option<&str> {
        self.attr("value")
    }
}

/// A small reader for the exported XML: elements, attributes and text.
pub fn parse_xml(s: &str) -> Option<Node> {
    let mut stack = vec![Node::default()];
    let mut rest = s;
    while let Some(lt) = rest.find('<') {
        let text = rest[..lt].trim();
        if !text.is_empty() {
            stack.last_mut()?.text.push_str(text);
        }
        rest = &rest[lt + 1..];
        let gt = rest.find('>')?;
        let tag = &rest[..gt];
        rest = &rest[gt + 1..];
        if tag.starts_with('?') || tag.starts_with('!') {
            continue;
        }
        if let Some(_close) = tag.strip_prefix('/') {
            let n = stack.pop()?;
            stack.last_mut()?.kids.push(n);
            continue;
        }
        let (body, empty) = match tag.strip_suffix('/') {
            Some(b) => (b, true),
            None => (tag, false),
        };
        let (name, mut a) = body.split_once(char::is_whitespace).unwrap_or((body, ""));
        let mut n = Node { tag: name.to_string(), ..Default::default() };
        while let Some((k, r)) = a.split_once("=\"") {
            let Some((v, r2)) = r.split_once('"') else { break };
            n.attrs.push((k.trim().to_string(), v.to_string()));
            a = r2;
        }
        if empty {
            stack.last_mut()?.kids.push(n);
        } else {
            stack.push(n);
        }
    }
    let mut top = stack.pop()?;
    top.kids.pop()
}

pub enum Val {
    Bool(bool),
    Float(f32),
    Int(i32),
    Vec3([f32; 3]),
    /// a value the file holds by hash (a part's or a mask's name), or a string
    Name(String),
}

pub struct SentParam {
    /// the hash of the parameter's name, or None for the kinds with no name in the file
    pub hash: u32,
    pub label: String,
    pub val: Val,
}

pub struct SentMessage {
    pub hash: u32,
    pub label: String,
    pub params: Vec<SentParam>,
    pub task_side: bool,
    /// parameters the file holds but nothing in NM takes from it (priority, blend, a reset of the message)
    pub other: Vec<String>,
}

pub struct Set {
    /// owner, path, the set's Id: each a name or 0x hash
    pub path: Vec<String>,
    pub enabled: bool,
    pub messages: Vec<SentMessage>,
}

fn hash_of(s: &str) -> Option<u32> {
    let s = s.strip_prefix("UNK_MEMBER_").unwrap_or(s);
    u32::from_str_radix(s.strip_prefix("0x")?, 16).ok()
}

fn name_hash(s: &str) -> u32 {
    hash_of(s).unwrap_or_else(|| joaat(s))
}

fn message(m: &Node) -> SentMessage {
    let raw = m.kid("Name").map_or("", |n| n.text.as_str());
    let mut out = SentMessage { hash: name_hash(raw), label: raw.to_string(), params: vec![], other: vec![],
                                task_side: m.kid("TaskMessage").and_then(Node::value) == Some("true") };
    for p in m.kid("Params").map_or(&[][..], |p| &p.kids) {
        let ty = p.attr("type").unwrap_or("");
        let name = p.kid("Name").map(|n| n.text.clone());
        let v = p.kid("Value");
        let num = |n: &Node, k: &str| n.attr(k).and_then(|x| x.parse::<f32>().ok()).unwrap_or(0.0);
        let val = match ty {
            "CNmParameterBool" => v.and_then(Node::value).map(|x| Val::Bool(x == "true")),
            "CNmParameterFloat" => v.and_then(Node::value).and_then(|x| x.parse().ok()).map(Val::Float),
            "CNmParameterInt" => v.and_then(Node::value).and_then(|x| x.parse().ok()).map(Val::Int),
            "CNmParameterVector" => v.map(|n| Val::Vec3([num(n, "x"), num(n, "y"), num(n, "z")])),
            // the game draws these; the middle of the range here, so that a press stays one stream
            "CNmParameterRandomFloat" => match (p.kid("Min").and_then(Node::value), p.kid("Max").and_then(Node::value)) {
                (Some(a), Some(b)) => Some(Val::Float(0.5 * (a.parse::<f32>().unwrap_or(0.0) + b.parse::<f32>().unwrap_or(0.0)))),
                _ => None,
            },
            "CNmParameterRandomInt" => match (p.kid("Min").and_then(Node::value), p.kid("Max").and_then(Node::value)) {
                (Some(a), Some(b)) => Some(Val::Int((a.parse::<i32>().unwrap_or(0) + b.parse::<i32>().unwrap_or(0)) / 2)),
                _ => None,
            },
            _ => None,
        };
        // the types with their own field: Start; the mask (a hash); Priority and Blend
        if let Some(s) = p.kid("Start").and_then(Node::value) {
            out.params.push(SentParam { hash: joaat("start"), label: "start".into(), val: Val::Bool(s == "true") });
        } else if let Some(pr) = p.kid("Priority").and_then(Node::value) {
            out.other.push(format!("Priority={pr}"));
        } else if let Some(pr) = p.kid("Blend").and_then(Node::value) {
            out.other.push(format!("Blend={pr}"));
        } else if ty == "CNmParameterResetMessage" {
            out.other.push("reset of the message".into());
        } else if let Some(n) = p.kids.iter().find(|k| k.tag == "UNK_MEMBER_0xFE9CFFBA") {
            out.params.push(SentParam { hash: joaat("mask"), label: "mask".into(), val: Val::Name(n.text.clone()) });
        } else if let Some(name) = name {
            let val = val.unwrap_or_else(|| Val::Name(v.map_or(String::new(), |n| n.text.clone())));
            out.params.push(SentParam { hash: name_hash(&name), label: name, val });
        } else {
            out.other.push(format!("{ty}?"));
        }
    }
    out
}

fn walk(n: &Node, trail: &mut Vec<String>, out: &mut Vec<Set>) {
    if let Some(ms) = n.kid("Messages") {
        let mut path = trail.clone();
        if let Some(id) = n.kid("Id") {
            path.push(id.text.clone());
        }
        out.push(Set { path, enabled: n.kid("Enabled").and_then(Node::value) != Some("false"),
                       messages: ms.kids.iter().filter(|m| m.attr("type") == Some("CNmMessage")).map(message).collect() });
    }
    for k in n.kids.iter().filter(|k| k.tag != "Messages" && !k.kids.is_empty()) {
        let label = if k.tag == "Item" { k.kid("Name").or(k.kid("Id")).map_or("Item".to_string(), |x| x.text.clone()) } else { k.tag.clone() };
        trail.push(label);
        walk(k, trail, out);
        trail.pop();
    }
}

/// Every message set in the file; the path starts with the tuning entry's name.
pub fn sets(file: &str) -> Option<Vec<Set>> {
    let root = parse_xml(file)?;
    let mut out = vec![];
    for it in &root.kid("Tunables")?.kids {
        let mut trail = vec![it.kid("Name").map_or(String::new(), |n| n.text.clone())];
        walk(it, &mut trail, &mut out);
    }
    Some(out)
}

/// The file in the folder: an exported physicstasks.
pub fn read_file(dir: &str) -> Result<String, String> {
    let d = std::fs::read_dir(dir).map_err(|e| format!("{dir}: {e}"))?;
    for e in d.flatten() {
        let n = e.file_name().to_string_lossy().to_lowercase();
        if n.starts_with("physicstasks") && n.ends_with(".xml") {
            return std::fs::read_to_string(e.path()).map_err(|x| x.to_string());
        }
    }
    Err(format!("no physicstasks*.xml in {dir}"))
}

/// Does each '/' piece of the selector name a piece of the path, in order? A piece is a name or a 0x hash (any case).
pub fn selects(sel: &str, path: &[String]) -> bool {
    let mut at = path.iter();
    sel.split('/').filter(|s| !s.is_empty()).all(|want| {
        let h = want.strip_prefix('?').map_or_else(|| hash_of(want).or(u32::from_str_radix(want, 16).ok().filter(|_| want.len() == 8)), |x| u32::from_str_radix(x, 16).ok());
        at.any(|p| p.eq_ignore_ascii_case(want) || hash_of(p).is_some_and(|ph| ph == joaat(want) || Some(ph) == h))
    })
}

/// The port's own message for the game's. The same name by hash, or one of the port's own names for it.
pub fn port_message(hash: u32) -> Option<&'static Message> {
    let alias = [("sit", "rdr2Sit"), ("configurebolas", "rdr2Bolas"), ("configureshotinjuredleg", "shotInjuredLeg"), ("configureshotinjuredarm", "shotInjuredArm")];
    let name = alias.iter().find(|a| joaat(a.0) == hash).map(|a| a.1);
    let all = || delirium::msg::MESSAGES.iter().chain(delirium::extras::messages());
    match name {
        Some(n) => all().find(|m| m.name == n),
        None => all().find(|m| joaat(m.name) == hash),
    }
}

/// The message line for one game message; and what the port doesn't take.
pub fn line(m: &SentMessage, port: &Message) -> (String, Vec<String>) {
    let mut s = port.name.to_string();
    let mut left = m.other.clone();
    for p in &m.params {
        if p.hash == joaat("start") {
            if let Val::Bool(b) = p.val {
                s += &format!(" start={b}");
            }
            continue;
        }
        let Some(prm) = port.params.iter().find(|x| joaat(x.name) == p.hash) else {
            left.push(format!("{} (no such parameter)", p.label));
            continue;
        };
        let txt = match (&p.val, prm.kind) {
            (Val::Bool(b), Kind::Bool) => b.to_string(),
            (Val::Bool(b), Kind::Int) => (*b as i32).to_string(),
            (Val::Bool(b), Kind::Float) => (*b as i32).to_string(),
            (Val::Float(x), Kind::Float) => x.to_string(),
            (Val::Float(x), Kind::Int) => (*x as i32).to_string(),
            (Val::Int(i), Kind::Int) => i.to_string(),
            (Val::Int(i), Kind::Float) => (*i as f32).to_string(),
            (Val::Int(i), Kind::Bool) => (*i != 0).to_string(),
            (Val::Vec3(v), Kind::Vec3) => format!("{},{},{}", v[0], v[1], v[2]),
            (Val::Name(n), Kind::Str) => match hash_of(n) {
                Some(h) => match ["fb", "ub", "lb", "us", "un", "uk", "ua", "ul", "ur", "uc", "uw", "ll", "lr", "la", "ra", "ls", "rs"].iter().find(|w| joaat(w) == h) {
                    Some(w) => w.to_string(),
                    None => { left.push(format!("{} = {n} (a name not known here)", p.label)); continue; }
                },
                None => n.clone(),
            },
            (Val::Name(n), _) => { left.push(format!("{} = {n} (held by hash)", p.label)); continue; }
            _ => { left.push(format!("{} (another type here)", p.label)); continue; }
        };
        s += &format!(" {}={txt}", prm.name);
    }
    (s, left)
}
