^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package potential_fields_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2026-04-04)
------------------
* Moved HSVtoRGB to pfield_manager instead
* Estimate Robot Geometry using Ellipsoids and Control Points for smooth WBV Repulsion (`#42 <https://github.com/argallab/potential_fields/issues/42>`_)
  Co-authored-by: Claude Sonnet 4.6 <noreply@anthropic.com>
* Updated README to fix example
* 40 test main branch ruleset (testing Issue `#40 <https://github.com/argallab/potential_fields/issues/40>`_) (`#41 <https://github.com/argallab/potential_fields/issues/41>`_)
* README edits and CONTRIBUTING.md
* Reflecting new RK4 estimation in unit test
* pfields_2025 -> potential_fields in docs and example scripts
* Cleaned plotting with more clear plots and messaging for invalid data
* Don't approximate joint velocities since people might try to use them
* Created visualization method to visualize both TS and WBV
* Removed link clearances since already computed in CSV creation
* Removed link clearances since already computed in CSV creation
* Populating csv with attraction force, repulsive force, and clearance
* Fixed cylinder distance bug
* Linting
* Fixed CSV bugs and tested demo1 and demo2, TS planning has problems
* 1.0.1
* Rename Packages (`#35 <https://github.com/argallab/potential_fields/issues/35>`_)
* Contributors: Demiana Barsoum, Sharwin Patil, Sharwin24
