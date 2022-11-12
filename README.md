# Vmd module for Noesis
A module that allows noesis scripts to export a vmd file upon loading a animation.

Currently supports motions and morphs.
Dictionaries allow for bones to automatically be renamed on export (will come with the model script).

## Options
Option  | Description
------------- | -------------
isMmdCam  | True = export native MMD camera vmd, False = export normal vmd.
fovThreshold  | (For MMD cams) If the camera moves on the next frame above this number on any axis, the fov will change.
cleanupThreshold  | (For MMD cams) If the difference between the current and both the previous and next frame is below this number, the keyframe will be skipped.