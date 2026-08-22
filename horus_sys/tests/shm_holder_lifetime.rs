//! A region must outlive its creator while anyone else still has it mapped.
//!
//! "Owner" only records who won the race to create the file. Unlinking on that
//! basis alone breaks the ordinary node restart: a publisher that created the
//! topic exits, takes the file with it, and every subscriber still mapped keeps
//! polling an orphaned inode that nobody will write to again. The next publisher
//! creates a fresh file and the two groups never meet.
//!
//! It reproduces only when the publisher happens to be the creator, which is why
//! it looks intermittent — run the subscriber first and everything works.

use horus_sys::shm::ShmRegion;

fn unique(prefix: &str) -> String {
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    )
}

#[test]
fn creator_dropping_does_not_unlink_while_another_holder_is_mapped() {
    let name = unique("holder_lifetime");
    let path = horus_sys::shm::topic_shm_path(&name);

    let creator = ShmRegion::new(&name, 4096).expect("create");
    let joiner = ShmRegion::open_existing(&name, 4096).expect("join");
    assert!(path.exists(), "sanity: region exists once created");

    // The creator leaves first — the common case when a publisher restarts.
    drop(creator);

    assert!(
        path.exists(),
        "the region was unlinked while another holder still had it mapped; a \
         subscriber that outlives its publisher would be left on an orphaned \
         inode, receiving nothing and reporting no error"
    );

    // Last one out does clean up.
    drop(joiner);
    assert!(
        !path.exists(),
        "the region should be removed once the last holder drops it"
    );
}

#[test]
fn sole_creator_still_cleans_up_on_drop() {
    let name = unique("holder_solo");
    let path = horus_sys::shm::topic_shm_path(&name);
    let region = ShmRegion::new(&name, 4096).expect("create");
    assert!(path.exists());
    drop(region);
    assert!(
        !path.exists(),
        "a region with no other holders must still be unlinked, or /dev/shm \
         accumulates a file per topic per run"
    );
}
