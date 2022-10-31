#!/bin/bash

if [ "$(type -t add_rosrs_setup_env)" == "function" ]; then
  # Collect the possible values for robocup scenario by checking which world files are present
  TMP_VITAL_RADAR_SCENARIOS_PATH=$(rospack find vital_radar_scenarios_description)
  if [ $? -eq 0 ]; then
    TMP_VITAL_RADAR_MAPS=()
    for f in ${TMP_VITAL_RADAR_SCENARIOS_PATH}/worlds/*.world; do
      TMP_VITAL_RADAR_MAPS+=($(basename ${f%.world}))
    done
    # The double @@ is necessary because catkin parses this file and uses @@ as a control character
    add_rosrs_setup_env VITAL_RADAR_MAP_ID "$(echo "${TMP_VITAL_RADAR_MAPS[@@]}" | tr ' ' ',')" "The map used for simulation if the robocup scenario is selected."
    unset TMP_VITAL_RADAR_MAPS
  fi
  unset TMP_VITAL_RADAR_SCENARIOS_PATH
fi

