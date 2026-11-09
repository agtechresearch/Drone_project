#!/usr/bin/env python3
"""Survey route text -> segments (v3_marker, 2026-11-01, codex survey-height-syntax).
grid_mission.survey_route used to split the whole spec at every comma, so the documented height list
'R1C W 10 1 0.68,1.265' became two segments ('R1C W 10 1 0.68' and a bare '1.265') and failed with "start cell must be the
first segment". Rule here: a comma separates segments, EXCEPT that a piece that is only a number continues the height list
of the segment before it. That segment must have the full form '<room><col> <W|E> <row_a> <row_b> <h1>'.
  split_segments(spec) -> ['R1C W 10 1 0.68,1.265', 'R1C E']        errors: SpecError with the offending piece
  parse_segment(seg)   -> (rc, side, rows or None, heights or None)  shape + number checks only (no map lookups)"""
import math


class SpecError(ValueError):
    pass


def _num(p):
    try:
        float(p)
        return True
    except ValueError:
        return False


def split_segments(spec):
    segs = []
    for p in (x.strip() for x in spec.split(",")):
        if not p:
            raise SpecError("empty segment in %r (two commas in a row, or a comma at the start / end)" % spec)
        if _num(p):
            if not segs or len(segs[-1].split()) != 5:
                raise SpecError("height %r has no segment to belong to: a height list needs the full form "
                                "'<room><col> <W|E> <row_a> <row_b> <h1>,<h2>,...' (in %r)" % (p, spec))
            segs[-1] += "," + p
        elif segs and len(p.split()) == 1:
            raise SpecError("%r after a segment is neither a height (number) nor a segment '<room><col> <W|E> ...'; a bare "
                            "start cell can only be the first segment (in %r)" % (p, spec))
        else:
            segs.append(p)
    if not segs:
        raise SpecError("empty survey spec")
    return segs


def parse_heights(tok):
    hs = []
    for h in tok.split(","):
        try:
            v = float(h)
        except ValueError:
            raise SpecError("height %r is not a number (heights: '0.68,1.265')" % h)
        if not math.isfinite(v) or v <= 0:
            raise SpecError("height %r must be a finite number > 0 (metres)" % h)
        hs.append(v)
    return hs


def parse_segment(seg):
    parts = seg.split()
    if len(parts) == 1:
        return parts[0].upper(), None, None, None                     # a bare start cell
    if len(parts) not in (2, 4, 5):
        raise SpecError("survey segment %r: '<room><col> <W|E> [row_a row_b] [h1,h2,...]' (rows come as a pair; "
                        "heights only after the rows)" % seg)
    side = parts[1].upper()
    if side not in ("W", "E"):
        raise SpecError("survey side must be W or E (the rack the camera looks at), got %r in %r" % (parts[1], seg))
    rows = None
    if len(parts) >= 4:
        try:
            rows = [int(parts[2]), int(parts[3])]
        except ValueError:
            raise SpecError("rows must be whole numbers in %r" % seg)
    heights = parse_heights(parts[4]) if len(parts) == 5 else None
    return parts[0].upper(), side, rows, heights
