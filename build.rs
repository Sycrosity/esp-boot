#[cfg(any(
    feature = "esp32",
    feature = "esp32c2",
    feature = "esp32c3",
    feature = "esp32c6",
    feature = "esp32h2",
    feature = "esp32s2",
    feature = "esp32s3",
))]
fn main() -> Result<(), String> {


    #[cfg(feature = "esp32")]
    println!("cargo:rustc-cfg=esp32");

    #[cfg(feature = "esp32c2")]
    println!("cargo:rustc-cfg=esp32c2");

    #[cfg(feature = "esp32c3")]
    println!("cargo:rustc-cfg=esp32c3");

    #[cfg(feature = "esp32c6")]
    println!("cargo:rustc-cfg=esp32c6");

    #[cfg(feature = "esp32h2")]
    println!("cargo:rustc-cfg=esp32h2");

    #[cfg(feature = "esp32s2")]
    println!("cargo:rustc-cfg=esp32s2");

    #[cfg(feature = "esp32s3")]
    println!("cargo:rustc-cfg=esp32s3");

    let version_output = std::process::Command::new(
        std::env::var_os("RUSTC").unwrap_or_else(|| std::ffi::OsString::from("rustc")),
    )
    .arg("-V")
    .output()
    .unwrap()
    .stdout;
    let version_string = String::from_utf8_lossy(&version_output);

    // HACK: we detect the xtensa-enabled compiler by existence of the second version string in parens
    // - upstream output format: rustc 1.75.0-nightly (cae0791da 2023-10-05)
    // - xtensa output format: rustc 1.73.0-nightly (9163a2087 2023-10-03) (1.73.0.0)
    // - gentoo format (non-xtensa): rustc 1.73.0-nightly (cc66ad468 2023-10-03) (gentoo)
    if version_string.chars().filter(|&c| c == '(').count() == 2 {
        let version = version_string
            .split('(')
            .last()
            .unwrap()
            .split(')')
            .next()
            .unwrap();
        if let Some(version) = try_read_xtensa_rustc_version(version) {
            if version >= Version4(1, 73, 0, 1)
                // Patch accidentally missing from 1.74.0.0
                && version != Version4(1, 74, 0, 0)
            {
                println!("cargo:rustc-cfg=xtensa_has_vaarg");
            }
        }
    }

    Ok(())

}

#[cfg(not(any(
    feature = "esp32",
    feature = "esp32c2",
    feature = "esp32c3",
    feature = "esp32c6",
    feature = "esp32h2",
    feature = "esp32s2",
    feature = "esp32s3",
)))]
fn main() {
    panic!("Select a chip via it's cargo feature");
}

fn try_read_xtensa_rustc_version(version: &str) -> Option<Version4> {
    let mut version = version.trim_start_matches('v').split('.');

    let major = version.next()?.parse::<u32>().ok()?;
    let minor = version.next()?.parse::<u32>().ok()?;
    let patch = version.next()?.parse::<u32>().ok()?;
    let release = version.next()?.parse::<u32>().ok()?;

    Some(Version4(major, minor, patch, release))
}

use std::cmp::Ordering;

#[derive(Debug, Clone, Copy, PartialEq)]
struct Version4(u32, u32, u32, u32);

impl PartialOrd for Version4 {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match self.0.partial_cmp(&other.0) {
            Some(Ordering::Equal) => {}
            ord => return ord,
        }
        match self.1.partial_cmp(&other.1) {
            Some(Ordering::Equal) => {}
            ord => return ord,
        }
        match self.2.partial_cmp(&other.2) {
            Some(Ordering::Equal) => {}
            ord => return ord,
        }
        self.3.partial_cmp(&other.3)
    }
}
