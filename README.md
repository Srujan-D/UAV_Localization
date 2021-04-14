# UAV_Localization
My implementations for filters used for state estimation and localization.

Currently has ekf implementation in C++ (branch- ekf). It is incomplete as of now. 

TODO:
- [ ] Figure out how to call the prediction and update functions from the sensor callbacks.
- [ ] Add white guasssian noise in prediction cov.
- [ ] Publish errors of the pose estimates and the gazebo odom so that we can judge the filter. Can use plotjuggler for this.
- [ ] To include either yaml file or use parameter servers for ease of tuning.
