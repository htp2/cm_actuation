So what's the deal with all these ambf launch files with setup / crtk / separate, etc.??

Basically:
- "setup" just loads things into AMBF
- "crtk" calls setup and also calls python scripts that ros-wrap the AMBF simulated robots to "look" like CRTK-compliant robots (i.e. subs/pubs for measured_js, measured_cp, etc.)
- "separate" loads all of the components separately (UR then actuation unit then connection between them). This is a "feature" due to modularity desires (e.g. switch out to UR10) but also a necessary requirement to get the ordering of joints correct if you are going to use the python crtk codes to do control