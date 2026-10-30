"""Small board texts in the KiCad 10 and KiCad 9 formats."""

# KiCad 10: nets referenced by name, (locked yes), no net table.
BOARD_V10 = r'''(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(generator_version "10.0")
	(layers
		(0 "F.Cu" signal)
		(4 "In1.Cu" power)
		(2 "B.Cu" signal)
	)
	(footprint "R_0603"
		(layer "F.Cu")
		(uuid "f0000000-0000-0000-0000-000000000001")
		(at 10 10)
		(pad "1" smd roundrect
			(at -0.8 0)
			(size 0.8 0.9)
			(layers "F.Cu")
			(net "GND")
			(uuid "f0000000-0000-0000-0000-000000000002")
		)
		(pad "2" smd roundrect
			(at 0.8 0)
			(size 0.8 0.9)
			(layers "F.Cu")
			(net "/sheet/SIG \"A\"")
			(uuid "f0000000-0000-0000-0000-000000000003")
		)
	)
	(segment
		(start 1 2)
		(end 3 4.5)
		(width 0.25)
		(layer "F.Cu")
		(net "GND")
		(uuid "a0000000-0000-0000-0000-000000000001")
	)
	(segment
		(start 3 4.5)
		(end 5 4.5)
		(width 0.5)
		(locked yes)
		(layer "B.Cu")
		(net "/sheet/SIG \"A\"")
		(uuid "a0000000-0000-0000-0000-000000000002")
	)
	(arc
		(start 0 0)
		(mid 0.7071 0.2929)
		(end 1 1)
		(width 0.2)
		(layer "F.Cu")
		(net "GND")
		(uuid "a0000000-0000-0000-0000-000000000003")
	)
	(via
		(at 7 8)
		(size 0.6)
		(drill 0.3)
		(layers "F.Cu" "B.Cu")
		(locked yes)
		(net "GND")
		(uuid "a0000000-0000-0000-0000-000000000004")
	)
	(via
		(at 9 9)
		(size 0.8)
		(drill 0.4)
		(layers "F.Cu" "In1.Cu")
		(net "GND")
		(uuid "a0000000-0000-0000-0000-000000000005")
	)
	(zone
		(net "GND")
		(layer "In1.Cu")
		(uuid "z0000000-0000-0000-0000-000000000001")
	)
)
'''

# KiCad 9 and older: net table, (net N), bare "locked" flag.
BOARD_V9 = '''(kicad_pcb (version 20240108) (generator "pcbnew")
  (net 0 "")
  (net 1 "GND")
  (net 2 "VCC")
  (segment locked (start 0 0) (end 1 0) (width 0.3) (layer "F.Cu") (net 2) (uuid "b1"))
  (segment (start 1 0) (end 2 0) (width 0.3) (layer "B.Cu") (net 1) (uuid "b2"))
  (segment (start 2 0) (end 3 0) (width 0.3) (layer "B.Cu") (net 0) (uuid "b3"))
  (via (at 2 0) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1) (uuid "b4"))
)
'''

NEW_ITEMS = '''	(segment
		(start 10 10)
		(end 12 10)
		(width 0.25)
		(layer "B.Cu")
		(net "GND")
		(uuid "c0000000-0000-0000-0000-000000000001")
	)
	(arc
		(start 12 10)
		(mid 12.5 10.2)
		(end 13 11)
		(width 0.25)
		(layer "B.Cu")
		(net "GND")
		(uuid "c0000000-0000-0000-0000-000000000002")
	)
	(via
		(at 13 11)
		(size 0.6)
		(drill 0.3)
		(layers "F.Cu" "B.Cu")
		(net "/sheet/SIG \\"A\\"")
		(uuid "c0000000-0000-0000-0000-000000000003")
	)
'''


def routed_v10(keep_locked: bool) -> str:
    """What `gummiband route` writes for BOARD_V10: the board without the replaced copper
    (unlocked items, or all without keep_locked), plus NEW_ITEMS at the end."""
    body = BOARD_V10.rstrip()
    assert body.endswith(")")
    blocks = body[:-1].rstrip().split("\n\t(")
    kept = [blocks[0]]
    for b in blocks[1:]:
        if b.startswith(("segment", "arc", "via")):
            if not (keep_locked and "(locked yes)" in b):
                continue
        kept.append(b)
    return "\n\t(".join(kept) + "\n" + NEW_ITEMS + ")\n"
