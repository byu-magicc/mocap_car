#include "path_follower_example.h"


namespace car_autopilot
{

path_follower_example::path_follower_example()
{
}

void path_follower_example::follow(const params_s &params, const input_s &input, output_s &output)
{
  if (input.p_type == path_type::Line) // follow straight line path specified by r and q
  {
    // compute wrapped version of the path angle
    float psi_q = atan2f(input.q_path[1], input.q_path[0]);
    while (psi_q - input.psi < -M_PI)
      psi_q += 2.0*M_PI;
    while (psi_q - input.psi > M_PI)
      psi_q -= 2.0*M_PI;

    float path_error = -sinf(psi_q)*(input.pn - input.r_path[0]) + cosf(psi_q)*(input.pe - input.r_path[1]);
    // heading command
    output.psi_c = psi_q - params.psi_infty*2/M_PI*atanf(params.k_path*path_error);

  }
  else // follow a orbit path specified by c_orbit, rho_orbit, and lam_orbit
  {
    float d = sqrtf(powf((input.pn - input.c_orbit[0]), 2) + powf((input.pe - input.c_orbit[1]),
                    2)); // distance from orbit center
    // compute wrapped version of angular position on orbit
    float varphi = atan2f(input.pe - input.c_orbit[1], input.pn - input.c_orbit[0]);
    while ((varphi - input.psi) < -M_PI)
      varphi += 2.0*M_PI;
    while ((varphi - input.psi) > M_PI)
      varphi -= 2.0*M_PI;
    //compute orbit error
    float norm_orbit_error = (d - input.rho_orbit)/input.rho_orbit;
    output.psi_c = varphi + input.lam_orbit*(M_PI/2.0 + atanf(params.k_orbit*norm_orbit_error));
  }
  output.u_c = input.u_d;
}

} //end namespace
