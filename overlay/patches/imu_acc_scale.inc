
    // --- benchmark patch: normalize accelerometer to m/s^2 -------------------
    // Some drivers report linear_acceleration in units of g (~1.0 at rest) --
    // e.g. the Livox /livox/imu stream in the Bunker DVI dataset. SR-LIO expects
    // m/s^2 (~9.8) and never rescales internally (unlike FAST-LIO, which scales
    // acc to the gravity magnitude at init), so a g-unit stream both inflates
    // the init noise check by (G_norm/|acc|)^2 ~ 96x ("Too large noise of
    // accelerometer measurements") and leaves an ~8.8 m/s^2 phantom acceleration
    // that makes the estimate diverge. Decide the scale once from the first
    // sample -- |acc| < 3 => the stream is in g => multiply by standard gravity.
    {
        static double imu_acc_scale = 0.0;
        if (imu_acc_scale == 0.0)
        {
            double an = std::sqrt(msg->linear_acceleration.x * msg->linear_acceleration.x +
                                  msg->linear_acceleration.y * msg->linear_acceleration.y +
                                  msg->linear_acceleration.z * msg->linear_acceleration.z);
            imu_acc_scale = (an < 3.0) ? 9.80665 : 1.0;
            ROS_INFO_STREAM("[imu] first |acc|=" << an << " -> acc_scale=" << imu_acc_scale);
        }
        msg_temp->linear_acceleration.x *= imu_acc_scale;
        msg_temp->linear_acceleration.y *= imu_acc_scale;
        msg_temp->linear_acceleration.z *= imu_acc_scale;
    }
    // --- end benchmark patch --------------------------------------------------
