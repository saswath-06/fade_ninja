"""A7/A8 analog: the spring keeps the heel on the scalp over the full sweep."""
import numpy as np

from fadegpt.head import SPRING_MAX_MM, SPRING_MIN_MM, Head
from fadegpt.interfaces import HeadGeometry


def _zone_grids(head: Head):
    g = head.geometry
    zone = (head.phi_grid >= g.phi_neckline_deg) & \
           (head.phi_grid <= g.phi_top_deg)
    return head.phi_grid[zone][:, None], head.psi_grid[None, :]


def test_a7_contact_holds_over_full_sweep():
    head = Head()
    phi, psi = _zone_grids(head)
    gap = head.gap(phi, psi)
    assert np.all(gap <= SPRING_MAX_MM), \
        f"contact lost: max gap {gap.max():.1f} mm > {SPRING_MAX_MM} mm reach"
    assert np.all(gap >= 0), "scalp pokes past the rail: geometry impossible"
    assert np.all(head.contact(phi, psi))


def test_a7_would_catch_a_misplaced_head():
    # head shifted 25 mm toward one ear: the far side falls out of spring reach
    head = Head(center_offset_mm=(0, 25, 0))
    phi, psi = _zone_grids(head)
    assert not np.all(head.contact(phi, psi)), \
        "sweep should FAIL for a badly placed head — that is what A7 is for"


def test_spring_travel_covers_head_ovality():
    # the head is an oval: gap variation over the zone must fit spring travel
    head = Head()
    phi, psi = _zone_grids(head)
    gap = head.gap(phi, psi)
    assert gap.max() - gap.min() < SPRING_MAX_MM - SPRING_MIN_MM, \
        "head ovality exceeds the 25 mm spring travel (gate A4)"


def test_cut_only_under_footprint():
    head = Head()
    head.cut(30.0, 90.0, leave_mm=3.0, width_mm=45.0, depth_mm=10.0)
    cut = head.cut_mask()
    assert cut.any()
    # far away untouched
    i_far = np.argmin(np.abs(head.phi_grid - 60))
    j_far = np.argmin(np.abs(head.psi_grid - 20))
    assert not cut[i_far, j_far]
    assert np.all(head.hair[cut] == 3.0)
    # cutting longer than what's left changes nothing
    before = head.hair.copy()
    head.cut(30.0, 90.0, leave_mm=8.0, width_mm=45.0, depth_mm=10.0)
    assert np.array_equal(head.hair, before)


def test_bigger_head_needs_its_own_rail_fit():
    small, big = Head(scale=1.0), Head(scale=1.1)
    assert big.rail_radius > small.rail_radius
    phi, psi = _zone_grids(big)
    assert np.all(big.contact(phi, psi))
