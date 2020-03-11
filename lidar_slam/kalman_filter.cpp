	void AttitudeEstimation::init(double g, double Q_deg_std_time_scale, double Q_deg_std_else, double R_uxy_deg_std, double R_uz_deg_std) {
		_pitch_rad = 0;
		_roll_rad = 0;
		X0 = angleToX(_pitch_rad, _roll_rad);

		_g = g;

		double init_pitch_roll_std = Localization::angle_math_utils::deg_to_rad(2);
		_confQRadStdTimeScale = Localization::angle_math_utils::deg_to_rad(Q_deg_std_time_scale);
		_confQRadStdElse = Localization::angle_math_utils::deg_to_rad(Q_deg_std_else);
		double R_uxy_std = Localization::angle_math_utils::deg_to_rad(R_uxy_deg_std);
		double R_uz_std = Localization::angle_math_utils::deg_to_rad(R_uz_deg_std);

		P0.MakeUnitMatrix(3);
		P0 = getR3Cov(init_pitch_roll_std, init_pitch_roll_std, _pitch_rad, _roll_rad);

		Q.MakeUnitMatrix(3);

		R.MakeUnitMatrix(3);
		R(0, 0) = R_uxy_std*R_uxy_std;
		R(1, 1) = R_uxy_std*R_uxy_std;
		R(2, 2) = R_uz_std*R_uz_std;

		Eye3.MakeUnitMatrix(3);
	}

    _attitudeEstimation->init(9.9, 0.1, 0.1, 2, 0.7);

    void AttitudeEstimation::doFilter(double gyro_deg_x, double gyro_deg_y, double gyro_deg_z, double acc_x, double acc_y, double acc_z, double t) {
		CMatrix gyro_deg(3,1);
		gyro_deg(0, 0) = gyro_deg_x;   gyro_deg(1, 0) = gyro_deg_y;   gyro_deg(2, 0) = gyro_deg_z;
		CMatrix acc(3, 1);
		acc(0, 0) = acc_x;   acc(1, 0) = acc_y;   acc(2, 0) = acc_z;
		doFilter(gyro_deg, acc, t);
	}

    void AttitudeEstimation::doFilter(CMatrix gyro_deg, CMatrix acc, double t) {//
		if (_inited == false) {
			return;
		}

		CMatrix AT(3, 3);
		Localization::rot_fuc_deg::d_rotmat_by_wt(gyro_deg(0, 0), gyro_deg(1, 0), gyro_deg(2, 0), t, AT.GetData());
		A = AT.Transpose();
		//原来有bug，原代码是
		//A=AT; 
		//A.Transpose(); 这句根本不起任何作用

		double p_rad, r_rad;
		xToAngle(X0, p_rad, r_rad);
		Q = getR3Cov(_confQRadStdTimeScale*t+_confQRadStdElse, _confQRadStdTimeScale*t+_confQRadStdElse, p_rad, r_rad);//getQFromDeltaT(t);

		CMatrix X1 = A*X0;
		CMatrix P1 = A*P0*AT + Q;

		CMatrix P1_Add_R = P1 + R;
		if (P1_Add_R.InvertGaussJordan() == false) {
			return;
		}
		CMatrix Kk = P1*P1_Add_R;

		CMatrix Z(3, 1);
		Z(0, 0) = acc(0, 0) / _g;   Z(1, 0) = acc(1, 0) / _g;   Z(2, 0) = acc(2, 0) / _g;

		X1 = X1 + Kk*(Z - X1);
		double x1_model = sqrt(X1(0, 0)*X1(0, 0) + X1(1, 0)*X1(1, 0) + X1(2, 0)*X1(2, 0));
		X0(0, 0) = X1(0, 0) / x1_model;   X0(1, 0) = X1(1, 0) / x1_model;   X0(2, 0) = X1(2, 0) / x1_model;

		P0 = (Eye3 - Kk)*P1;

		xToAngle(X0, _pitch_rad, _roll_rad);
	}