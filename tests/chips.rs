//! The chip list a front end offers, checked against the registry it comes from.
//!
//! **Needs no board and no ELF**, which is the point of it: a picker has to be populated before
//! anything is attached, and that is exactly the state a bench most often opens in.

/// The list is not empty, is sorted, and has no name twice.
///
/// **Sorted and deduplicated is a promise the doc comment makes**, and both are properties a
/// caller builds on: a picker renders the order it is given, and a duplicate would draw twice.
#[test]
fn the_list_is_sorted_and_unique() {
    let chips = probe_bench::chips();
    assert!(!chips.is_empty(), "the built-in target database decoded to nothing");
    assert!(chips.is_sorted(), "a caller renders this order");
    let mut unique = chips.clone();
    unique.dedup();
    assert_eq!(chips, unique, "a name twice would draw twice");
}

/// Every name in the list resolves to exactly the chip it names.
///
/// **Nothing offered is junk**, checked over the whole list rather than a sample, since the
/// failure would be per name. What this cannot tell you is whether the list is the *right* one —
/// see the test below, which is the half that distinguishes package variants from variant names.
#[test]
fn every_name_attaches_as_itself() {
    let registry = probe_rs::config::Registry::from_builtin_families();
    for name in probe_bench::chips() {
        let target = registry
            .get_target_by_name(&name)
            .unwrap_or_else(|e| panic!("{name} is offered and does not resolve: {e}"));
        // The registry names a resolved target after the package it matched, so this is the
        // check it looks like rather than a tautology: a name that resolved by prefix to some
        // other package comes back carrying that package's name.
        assert!(
            target.name.eq_ignore_ascii_case(&name),
            "{name} resolved to {}, which is a different part",
            target.name,
        );
    }
}

/// The registry's own search cannot serve a picker, and this is the evidence rather than a claim.
///
/// **`search_chips` is a prefix match with `x` as a single-character wildcard.** A person typing
/// the distinctive middle of a part number gets nothing back, which is the behaviour a filter in
/// a list box must not have — so a front end takes [`probe_bench::chips`] and narrows it itself.
#[test]
fn the_registrys_search_is_a_prefix_match() {
    let registry = probe_rs::config::Registry::from_builtin_families();
    let Some(full) = probe_bench::chips().into_iter().find(|n| n.len() > 4) else {
        panic!("no name long enough to take a prefix and a tail of");
    };

    assert!(
        !registry.search_chips(&full[..3]).is_empty(),
        "a prefix of {full} matched nothing, so this is not a prefix match either",
    );
    assert!(
        registry.search_chips(&full[2..]).iter().all(|found| found != &full),
        "{full} was found by its tail, so the search is not prefix-only and this test is stale",
    );
}

/// The list carries the names on the parts, not only the family's variant names.
///
/// **This is the half that says which list it is.** Every variant name is also a package name, so
/// a list of variant names resolves perfectly well and is simply 2,330 names short — including
/// every name somebody would read off the part in front of them. Nothing about a resolution check
/// can see that, which is why it is asserted separately.
#[test]
fn the_list_includes_the_package_names() {
    let registry = probe_rs::config::Registry::from_builtin_families();
    let variants: std::collections::BTreeSet<&str> = registry
        .families()
        .iter()
        .flat_map(|family| family.variants.iter())
        .map(|chip| chip.name.as_str())
        .collect();

    let offered = probe_bench::chips();
    assert!(
        offered.len() > variants.len(),
        "{} names offered against {} variant names: this is the variant list",
        offered.len(),
        variants.len(),
    );
    assert!(
        variants.iter().all(|name| offered.iter().any(|o| o == name)),
        "a variant name is missing, so the list is not a superset of them",
    );
}
