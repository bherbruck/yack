// yack: STEP assemblies (parts written once, placed by instances) and what a read says of each body.
// Adapted from springrep@a03e8a1 tests/step_assemblies.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); every test is omitted as an API gap:
// yack's `step` module writes and reads one Solid and has no assembly, body, instance or colour model.

// The original's fixture: a block 10 × 20 × 30 coloured RED [0.8, 0.1, 0.1], and a post (cylinder
// r 5, h 10) used twice — "post-a" moved 40 up z, "post-b" turned a quarter about x then moved 50
// along x — written by `step::write_assembly("stand", &parts, &instances)`.

// API GAP (test an_assembly_reads_back_with_each_instance_where_it_was_placed omitted): needs
// writing an assembly (parts + named placed instances, each part's geometry written once) and a
// read that reports each body's part name, instance names, the shared STEP id of a part's
// instances and each body's own bounding box; yack's step::write/read carry one Solid only.

// API GAP (test a_parts_colour_is_its_bodies_colour_wherever_they_are_placed omitted): needs an
// assembly writer with per-part colours and a read that reports each body's colour and the
// per-face colours.

// API GAP (test a_face_styled_apart_from_its_body_has_its_own_colour omitted): needs a read that
// reports per-face colours (STYLED_ITEM on an ADVANCED_FACE) mapped to the solid's FaceIds, and
// the bodies of the read with their faces and colours; the file itself also comes from the
// assembly writer.

// API GAP (test a_file_of_one_part_names_its_body_and_gives_it_no_use omitted): needs step::write to
// take a product name ("lone") and a read that reports bodies with their name, instances and
// colour; yack's step::write(&Solid) takes no name and step::read returns only the Solid.

// API GAP (test an_instance_of_a_part_the_file_lacks_is_invalid omitted): needs the assembly writer,
// whose refusal of an instance naming a missing part (ErrorKind::Invalid) is what is tested.
