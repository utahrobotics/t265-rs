# t265-rs

This crate is designed to handle low level communication with Intel T265 (and T261 probably) depth cameras over USB. 
The API it presents to users allows cameras to be grouped together, and for data to be received in convenient stream formats, such as `Result<crossbeam::channel::Receiver<VideoFrame>>`.
Supported data currently includes:
* Camera frames
* Pose
* IMU frames
* Temperature

*Note: This utility was written by Matthew A, but this README was written by Hale B. I don't know everything about the utility yet, so take all this with a grain of salt.*
