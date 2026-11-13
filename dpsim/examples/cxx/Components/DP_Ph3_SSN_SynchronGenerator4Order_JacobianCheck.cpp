// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_SSN_SynchronGenerator4Order.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace CPS;

namespace CPS::DP::Ph3::SSN {
class SynchronGenerator4OrderJacobianCheck {
public:
  static void run() {
    constexpr Real nominalPower = 100e6;
    constexpr Real nominalVoltage = 230e3;
    constexpr Real baseVoltage = nominalVoltage * 0.8164965809277260327;

    auto machine = SynchronGenerator4Order::make("jacobian_check");
    machine->setParametersOperationalPerUnit(nominalPower, nominalVoltage, 50.,
                                             3.5, 0.003, 1.8, 1.7, 1.9, 1.8, 8.,
                                             0.4, 1.);

    Real maxScaledError = 0.;
    for (Int test = 0; test < 3; ++test) {
      Matrix x(4, 1);
      x << 0.92 + 0.03 * test, 1.08 - 0.02 * test, 0.98 + 0.01 * test,
          -2.1 + 1.25 * test;

      const Complex va(baseVoltage * (0.91 + 0.02 * test),
                       baseVoltage * (0.07 - 0.01 * test));
      const Complex vb = va * SHIFT_TO_PHASE_B +
                         Complex(baseVoltage * 0.013, -baseVoltage * 0.009);
      const Complex vc = va * SHIFT_TO_PHASE_C +
                         Complex(-baseVoltage * 0.008, baseVoltage * 0.011);
      Matrix u(6, 1);
      u << va.real(), va.imag(), vb.real(), vb.imag(), vc.real(), vc.imag();

      Matrix A, B, C, D;
      machine->linearize(x, u, A, B, C, D);
      Matrix fp, fm, yp, ym;

      const auto recordError = [&](Real analytic, Real numerical) {
        maxScaledError =
            std::max(maxScaledError, std::abs(analytic - numerical) /
                                         std::max(1., std::abs(numerical)));
      };

      for (Int col = 0; col < 4; ++col) {
        const Real h = 1e-6 * std::max(1., std::abs(x(col, 0)));
        Matrix xp = x, xm = x;
        xp(col, 0) += h;
        xm(col, 0) -= h;
        machine->evaluate(xp, u, fp, yp);
        machine->evaluate(xm, u, fm, ym);
        for (Int row = 0; row < 4; ++row)
          recordError(A(row, col), (fp(row, 0) - fm(row, 0)) / (2. * h));
        for (Int row = 0; row < 6; ++row)
          recordError(C(row, col), (yp(row, 0) - ym(row, 0)) / (2. * h));
      }

      for (Int col = 0; col < 6; ++col) {
        const Real h = 1e-6 * std::max(1., std::abs(u(col, 0)));
        Matrix up = u, um = u;
        up(col, 0) += h;
        um(col, 0) -= h;
        machine->evaluate(x, up, fp, yp);
        machine->evaluate(x, um, fm, ym);
        for (Int row = 0; row < 4; ++row)
          recordError(B(row, col), (fp(row, 0) - fm(row, 0)) / (2. * h));
        for (Int row = 0; row < 6; ++row)
          recordError(D(row, col), (yp(row, 0) - ym(row, 0)) / (2. * h));
      }
    }

    if (maxScaledError > 2e-5)
      throw std::runtime_error("Analytic generator Jacobian mismatch: " +
                               std::to_string(maxScaledError));

    std::cout << "Analytic DP SSN generator Jacobian check passed; max scaled "
                 "error="
              << maxScaledError << '\n';
  }
};
} // namespace CPS::DP::Ph3::SSN

int main() { CPS::DP::Ph3::SSN::SynchronGenerator4OrderJacobianCheck::run(); }
