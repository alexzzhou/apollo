/*
@licstart  The following is the entire license notice for the
JavaScript code in this file.

Copyright (C) 1997-2019 by Dimitri van Heesch

This program is free software; you can redistribute it and/or modify
it under the terms of version 2 of the GNU General Public License as published by
the Free Software Foundation

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

@licend  The above is the entire license notice
for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "Xsens Device API", "index.html", [
    [ "Xsens Device API Library", "index.html", [
      [ "Introduction", "index.html#intro_xda", null ],
      [ "Introduction to the API", "index.html#s_overview", null ],
      [ "Example Code", "index.html#s_examples", null ],
      [ "ROS MTi driver", "index.html#s_rosdriver", null ]
    ] ],
    [ "Upgrading from XDA 4.x", "migration.html", [
      [ "Upgrading from XDA 4.0 to XDA 4.1 or higher", "migration.html#mig_40_41_intro", [
        [ "Breaking changes", "migration.html#mig_40_41_changes", null ],
        [ "New features", "migration.html#mig_40_41_new", null ]
      ] ],
      [ "Upgrading from XDA 4.4 to XDA 4.5 or higher", "migration.html#mig_44_45_intro", [
        [ "Breaking changes", "migration.html#mig_44_45_changes", null ],
        [ "New features", "migration.html#mig_44_45_new", null ]
      ] ],
      [ "Upgrading from XDA 4.6 to XDA 4.7 or higher", "migration.html#mig_46_47_intro", [
        [ "Breaking changes", "migration.html#mig_46_47_changes", null ],
        [ "New features", "migration.html#mig_46_47_new", null ]
      ] ],
      [ "Upgrading from XDA 4.7 to XDA 4.8.6 or higher", "migration.html#mig_47_50_intro", [
        [ "Breaking changes", "migration.html#mig_47_50_changes", null ],
        [ "New features", "migration.html#mig_47_50_new", null ]
      ] ]
    ] ],
    [ "Overview of the Xsens Device API", "overview.html", [
      [ "Introduction", "overview.html#s_introduction", null ],
      [ "Set-up", "overview.html#s_setup", [
        [ "Important Classes of the C++ interface", "overview.html#ss_mainClasses", null ],
        [ "Device Hierarchy", "overview.html#ss_deviceTree", null ]
      ] ],
      [ "Device Diversity", "overview.html#s_deviceDiversity", [
        [ "Program Flow", "overview.html#ss_programFlow", null ],
        [ "Concurrency", "overview.html#s_concurrency", null ]
      ] ]
    ] ],
    [ "Xsens Device API Objectives and Considerations", "p_background.html", [
      [ "Objectives", "p_background.html#s_objectives", null ],
      [ "Considerations", "p_background.html#s_considerations", [
        [ "Language", "p_background.html#ss_language", null ],
        [ "Name mangling", "p_background.html#ss_nameMangling", null ],
        [ "Generic types libraries", "p_background.html#ss_genericTypes", null ],
        [ "Generic, complexity and flexibility", "p_background.html#ss_generic", null ],
        [ "Used internally", "p_background.html#ss_usedInternally", null ]
      ] ]
    ] ],
    [ "Code Examples", "p_examples.html", [
      [ "XDA MTi Examples", "p_examples.html#s_xda_mti_ex", null ],
      [ "XDA MTw Examples", "p_examples.html#s_xda_mtw_ex", null ],
      [ "Built Examples", "p_examples.html#s_xda_built_ex", null ],
      [ "Embedded Examples", "p_examples.html#s_xda_embedded_ex", null ],
      [ "Questions?", "p_examples.html#s_question_ex", null ]
    ] ],
    [ "Workflow using the Xsens Awinda System", "awindaworkflow.html", [
      [ "Introduction", "awindaworkflow.html#s_intro", null ],
      [ "Awinda Master states", "awindaworkflow.html#s_awsstates", null ],
      [ "Mtw states", "awindaworkflow.html#s_mtwstates", null ],
      [ "Work Flow", "awindaworkflow.html#s_workflow", null ]
    ] ],
    [ "Public Xsens Device API", "p_publicxda.html", [
      [ "Introduction", "p_publicxda.html#s_publicXdaIntroduction", null ],
      [ "Overview", "p_publicxda.html#s_overview_pxda", null ],
      [ "Requirements", "p_publicxda.html#s_requirements_pxda", null ],
      [ "Getting started", "p_publicxda.html#s_gettingstarted_pxda", null ],
      [ "Questions?", "p_publicxda.html#s_question_pxda", null ]
    ] ],
    [ "Retransmissions and Stream behaviour for Xsens Awinda Systems", "awindaretransmissions.html", [
      [ "Summary", "awindaretransmissions.html#awindaretransmissions_summary", [
        [ "The Live Stream", "awindaretransmissions.html#awindaretransmissions_livestream", null ],
        [ "The Buffered Stream", "awindaretransmissions.html#awindaretransmissions_bufferedstream", null ],
        [ "Flushing Mode", "awindaretransmissions.html#awindaretransmissions_flushingmode", null ],
        [ "The Data Stream", "awindaretransmissions.html#awindaretransmissions_datastream", null ]
      ] ],
      [ "Multiple MTw behaviour", "awindaretransmissions.html#awindaretransmissions_multidev", [
        [ "The All Live Stream", "awindaretransmissions.html#awindaretransmissions_alllivestream", null ],
        [ "The All Buffered Stream", "awindaretransmissions.html#awindaretransmissions_allbufferedstream", null ],
        [ "The All Data Stream", "awindaretransmissions.html#awindaretransmissions_alldatastream", null ]
      ] ],
      [ "A note on delays in the Buffered Stream", "awindaretransmissions.html#awindaretransmissions_datastreamdelays", null ]
    ] ],
    [ "Xsens MTi driver for ROS", "p_rosdriver.html", [
      [ "Introduction", "p_rosdriver.html#s_intrt_rosdriver", null ],
      [ "Overview", "p_rosdriver.html#s_overview_rosdriver", null ],
      [ "Requirements", "p_rosdriver.html#s_requirements_rosdriver", null ],
      [ "Building the Xsens MTi node", "p_rosdriver.html#s_building_rosdriver", null ],
      [ "Running the Xsens MTi node", "p_rosdriver.html#s_running_rosdriver", null ],
      [ "Configuring the Xsens MTi node", "p_rosdriver.html#s_configuring_rosdriver", [
        [ "Published topics", "p_rosdriver.html#subs_topics_rosdriver", null ]
      ] ],
      [ "Troubleshooting", "p_rosdriver.html#s_troubleshooting_rosdriver", null ],
      [ "Questions?", "p_rosdriver.html#s_question_rosdriver", null ]
    ] ],
    [ "Deprecated List", "deprecated.html", null ],
    [ "Modules", "modules.html", "modules" ],
    [ "Namespace Members", "namespacemembers.html", [
      [ "All", "namespacemembers.html", null ],
      [ "Functions", "namespacemembers_func.html", null ]
    ] ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Functions", "functions_func.html", "functions_func" ],
        [ "Variables", "functions_vars.html", "functions_vars" ],
        [ "Typedefs", "functions_type.html", null ],
        [ "Related Functions", "functions_rela.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"annotated.html",
"group___callbacks.html#gaf94a5a76c3a443255fc53f7a2bf9e076",
"group__cinterface.html#ga42d5a40697d57c0da08e1d21c82fb6e6",
"group__cinterface.html#ga7b75e01c27f76b4d91aaa135e4e1a5c3",
"group__cinterface.html#gabcf714ff1335befc61d96f42a1d9a322",
"group__cinterface.html#gaff3c155ea6a5ed3e225257e706d5e9f0",
"group__enums.html#gga76e175ae236aa6c8badbf198f132b427affaa78852efb90f484f496593944c875",
"migration.html#mig_44_45_new",
"struct_xs_control.html#a383124de0bf073f72800eb1905008eee",
"struct_xs_device.html#a2bfd5957f51759bd28c2c9f29a1cd6bf",
"struct_xs_device_id.html#a61275b5bd4f76e27b05c47d1622aa017",
"struct_xs_finger_snapshot.html",
"struct_xs_quaternion.html#a2c30c404abee1cfee1d5ea742b4ff9ef",
"struct_xs_string.html#ae4e38b08f81789ab02035f033160767a"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';