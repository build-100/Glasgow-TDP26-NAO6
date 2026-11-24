# Third-party notices

The model package includes the selected `NAOH25V60.urdf` and its matching
mesh/material resources from RajasJoshi/UoG_Robotics_TDP4 at commit
`8a0949302aac5d610339fa02f3e4cfc437abb17d`. The source repository declares
Apache-2.0 at its root; a verbatim copy is retained as
`ros/nao6_description/LICENSE.upstream`. No separate licence header was found
in the selected URDF or text material file. This is an upstream licence
attribution, not a certification of the original vendor asset provenance.

Original and adapted model descriptions and mesh assets are excluded from the
root restricted-use notice and retain upstream terms. `source_manifest.json`
records original URLs and hashes. `adaptation_report.json` records modifications.

The [whole-body workbench](ros/nao6_description/control/whole_body/README.md)
adapts the motion selection/dispatch approach of upstream `Wenao/controllers/Utils/Consts.py`
and `RedForwardA/RedForwardA.py` at the same pinned revision. Its Python implementation
is written for this project. It loads seven original motion files, the NaoV6 PROTO
and its wrist submodels from a verified local cache or upstream checkout. Those
asset bytes are obtained on the user's machine. The PROTO headers specify
Cyberbotics' [Webots asset licence](https://cyberbotics.com/webots_assets_license).
Generated local worlds use those assets with Webots and retain their notices.

No private GEVD implementation, research manuscript or source course handout is
included. The planning scaffold and adaptation/validation scripts were written
for this course repository.
