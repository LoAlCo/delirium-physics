//! Builds a GTA 6 body on the RDR2 core (the player's own decoded GTA 6 ragdoll + RDR2's fred.xml or wilma.xml, on
//! Homer's body from their GTA IV), prints its masses and proportions, then stands it up and sends each GTA 6
//! behaviour to a fresh copy, printing how it ends up.
//! Usage: GTAIV_EXE=<GTAIV.exe> cargo run --release -p delirium-sdk --example gta6_body --
//!          <Fred.art|fred.xrt> <build.txt> <rdr2 body .xml> <gta6 ragdoll .json> <out.json> [base]
//! "base" at the end runs the behaviours on the plain base body instead, to compare.
use delirium_physics::character::*;
use delirium_physics::gta6::*;
use delirium_physics::*;
use std::ffi::CString;

fn c(s: &str) -> CString {
    CString::new(s).unwrap()
}

unsafe fn json(f: impl Fn(*mut std::ffi::c_char, i32) -> i32) -> String {
    let n = f(std::ptr::null_mut(), 0);
    assert!(n > 0, "{}", last());
    let mut b = vec![0u8; n as usize + 1];
    f(b.as_mut_ptr() as *mut _, b.len() as i32);
    String::from_utf8_lossy(&b).trim_end_matches('\0').to_string()
}

fn last() -> String {
    let mut b = vec![0u8; 512];
    unsafe { nm_last_error(b.as_mut_ptr() as *mut _, 512) };
    String::from_utf8_lossy(&b).trim_end_matches('\0').to_string()
}

unsafe fn part(g: *mut NmAgent, k: i32) -> [f32; 3] {
    let mut p = [0.0f32; 3];
    unsafe { nm_agent_get_part_transform(g, k, p.as_mut_ptr(), std::ptr::null_mut()) };
    p
}

/// A behaviour and its parameters: (name, ints, floats, bools, vectors).
type Try = (&'static str, &'static [(&'static str, i32)], &'static [(&'static str, f32)], &'static [(&'static str, bool)], &'static [(&'static str, [f32; 3])]);

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let on_base = a.get(6).is_some_and(|s| s == "base");
    unsafe {
        if let Ok(e) = std::env::var("GTAIV_EXE") {
            assert_eq!(nm_unlock(c(&e).as_ptr()), 0, "nm_unlock: {}", last());
        }
        if a[1].to_ascii_lowercase().ends_with(".art") {
            delirium_data::LE.store(true, std::sync::atomic::Ordering::Relaxed);
        }
        let (xrt, build, xml, rag) = (c(&a[1]), c(&a[2]), c(&a[3]), c(&a[4]));
        let base = json(|b, n| nm_character_from_xrt(xrt.as_ptr(), build.as_ptr(), b, n));
        let base_c = c(&base);
        let body = json(|b, n| nm_character_from_gta6(rag.as_ptr(), xml.as_ptr(), base_c.as_ptr(), build.as_ptr(), 1, b, n));
        std::fs::write(&a[5], &body).unwrap();
        println!("wrote {} ({} bytes)", a[5], body.len());
        let (bj, gj): (serde_json::Value, serde_json::Value) = (serde_json::from_str(&base).unwrap(), serde_json::from_str(&body).unwrap());
        let names = ["pelvis", "L thigh", "L shin", "L foot", "R thigh", "R shin", "R foot", "spine0", "spine1", "spine2", "chest",
                     "neck", "head", "L clav", "L upper arm", "L forearm", "L hand", "R clav", "R upper arm", "R forearm", "R hand"];
        println!("part           base kg  GTA 6 kg");
        for (k, n) in names.iter().enumerate() {
            println!("{n:14} {:7.2}  {:7.2}", bj["parts"][k]["mass"].as_f64().unwrap(), gj["parts"][k]["mass"].as_f64().unwrap());
        }
        let used = if on_base { &base } else { &body };
        let bc = c(used);
        assert!(nm_init_character(bc.as_ptr()) >= 0, "nm_init_character: {}", last());
        let b = nm_build_from_character(bc.as_ptr());
        assert!(!b.is_null(), "nm_build_from_character: {}", last());
        let w = nm_world_create();
        nm_world_add_floor(w, 0.0, 0.0, 0.0, 50.0);
        let tries: &[Try] = &[
            ("none", &[], &[], &[], &[]),
            ("gta6BulletImpact", &[("bodyPart", 10)], &[], &[], &[("bulletDir", [0.0, 1.0, 0.0])]),
            ("gta6BulletImpact", &[("bodyPart", 2), ("toughness", 1)], &[], &[], &[("bulletDir", [0.0, 1.0, 0.0])]),
            ("gta6ShotgunKnockdown", &[("area", 0)], &[], &[], &[("bulletDir", [0.0, 1.0, 0.0])]),
            ("gta6HitReact", &[("weapon", 0)], &[], &[], &[("hitNormal", [0.0, -1.0, 0.0])]),
            ("gta6HitReact", &[("weapon", 2), ("style", 2)], &[], &[], &[("hitNormal", [0.0, -1.0, 0.0])]),
            ("gta6RecoverBalance", &[], &[], &[], &[]),
            ("gta6Stumble", &[], &[], &[], &[]),
            ("gta6Stumble", &[], &[], &[("toKnees", true)], &[]),
            ("gta6Trip", &[], &[], &[], &[]),
            ("gta6LowSpeedImpact", &[("strength", 1)], &[], &[], &[("dir", [1.0, 0.0, 0.0])]),
            ("gta6LowSpeedImpact", &[("strength", 2)], &[], &[], &[("dir", [1.0, 0.0, 0.0])]),
            ("gta6Freefall", &[], &[], &[], &[]),
            ("gta6StuntFall", &[], &[], &[], &[]),
            ("gta6FallOverFence", &[], &[], &[], &[]),
            ("gta6Roll", &[], &[], &[], &[]),
            ("gta6Roll", &[], &[], &[("tucked", false)], &[]),
            ("gta6Writhe", &[], &[], &[], &[]),
            ("gta6Writhe", &[], &[], &[("dying", true)], &[]),
            ("gta6Burning", &[], &[], &[], &[]),
            ("gta6Electrocuted", &[], &[], &[], &[]),
            ("gta6VehicleTumble", &[], &[], &[], &[]),
            ("gta6VehicleTumble", &[], &[], &[("knockedOut", true)], &[]),
            ("gta6ReachForInjury", &[("hitPart", 8)], &[], &[], &[]),
        ];
        println!("\n{} body: each behaviour sent at 1 s, state at 4 s", if on_base { "base" } else { "GTA 6" });
        for (name, ints, floats, bools, vecs) in tries {
            let g = nm_agent_create(b, [0.0f32; 3].as_ptr(), 0.0, 0.3);
            nm_msg_begin(g, c("bodyBalance").as_ptr());
            nm_msg_set_bool(g, c("start").as_ptr(), 1);
            nm_msg_send(g, w);
            let (mut low, mut sent) = (f32::MAX, 0);
            for k in 0..240 {
                if k == 60 && *name != "none" {
                    nm_msg_begin(g, c(name).as_ptr());
                    nm_msg_set_bool(g, c("start").as_ptr(), 1);
                    for (n, v) in *ints { nm_msg_set_int(g, c(n).as_ptr(), *v); }
                    for (n, v) in *floats { nm_msg_set_float(g, c(n).as_ptr(), *v); }
                    for (n, v) in *bools { nm_msg_set_bool(g, c(n).as_ptr(), *v as i32); }
                    for (n, v) in *vecs { nm_msg_set_vec3(g, c(n).as_ptr(), v.as_ptr()); }
                    sent = nm_msg_send(g, w);
                    if sent < 0 {
                        println!("{name}: send failed: {}", last());
                    }
                }
                assert_eq!(nm_agent_step(g, w, 1.0 / 60.0), 0, "{name}: {}", last());
                low = low.min(part(g, 0)[2]);
            }
            let (pel, head) = (part(g, 0), part(g, 12));
            let opts: Vec<String> = ints.iter().map(|(n, v)| format!("{n}={v}")).chain(bools.iter().map(|(n, v)| format!("{n}={v}"))).collect();
            println!("{:22} {:24} pelvis z {:.2} (lowest {:.2})  head z {:.2}  moved {:.2} m  balancing {} fallen {}  [{}]",
                     name, opts.join(" "), pel[2], low, head[2], (pel[0].powi(2) + pel[1].powi(2)).sqrt(),
                     nm_agent_is_balancing(g), nm_agent_has_fallen(g), sent);
            nm_agent_destroy(g);
        }
    }
}
