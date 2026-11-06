//! Writes what each message reads and its default, one JSON line per read.
//! Usage: param_audit <out> <the GTA IV folder, for the numbers read from it>
fn main() {
    let mut a = std::env::args().skip(1);
    let out = a.next().expect("an output path");
    let dir = std::path::PathBuf::from(a.next().expect("the GTA IV folder"));
    let exe = ["GTAIV.exe", "default.xex"].iter().map(|f| dir.join(f)).find(|p| p.is_file()).expect("GTAIV.exe");
    assert!(delirium_data::kv::unlock(&delirium_data::xex::program(&exe).expect("the program")));
    delirium::extras::param_audit::dump(&out);
}
