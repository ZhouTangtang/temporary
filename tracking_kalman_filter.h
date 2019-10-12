/*!
 * \file tracking_FLL_PLL_filter.h
 * \brief Interface of a Kalman filter for tracking carrier loop
 * \author ZYX. Wuhan University
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_KALMAN_FILTER_H_
#define GNSS_SDR_KALMAN_FILTER_H_

/*!
 * \brief This class implements a hybrid FLL and PLL filter for tracking carrier loop
 */
class Tracking_Kalman_filter
{
public:
    void set_params(float fll_bw_hz, float pll_bw_hz, int order);
    void initialize(float d_acq_carrier_doppler_hz,float carrier_phase_rad);
    void update_carrier(float FLL_discriminator, float PLL_discriminator, float correlation_time_s);
    Tracking_Kalman_filter();
    ~Tracking_Kalman_filter() = default;

private:
    int d_order;
    // Kalman filter variables
    double f_hz;
    double phase_rad;
    double f_rate_hzps;

    //arma::mat kf_P_x_ini;  // initial state error covariance matrix
    //arma::mat kf_P_x;      // state error covariance matrix
    //arma::mat kf_P_x_pre;  // Predicted state error covariance matrix
    //arma::mat kf_P_y;      // innovation covariance matrix

    //arma::mat kf_F;  // state transition matrix
    //arma::mat kf_H;  // system matrix
    //arma::mat kf_R;  // measurement error covariance matrix
    //arma::mat kf_Q;  // system error covariance matrix

    //arma::colvec kf_x;      // state vector
    //arma::colvec kf_x_pre;  // predicted state vector
    //arma::colvec kf_y;      // measurement vector
    //arma::mat kf_K;         // Kalman gain matrix

    arma::mat P; // error state covariance
    arma::mat H; // system matrix
    arma::mat Phi; // transition matrix
    arma::colvec x; // state

};

#endif
