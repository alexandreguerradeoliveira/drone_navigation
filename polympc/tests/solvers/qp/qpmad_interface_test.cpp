// This file is part of PolyMPC, a lightweight C++ template library
// for real-time nonlinear optimization and optimal control.
//
// Copyright (C) 2020 Listov Petr <petr.listov@epfl.ch>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "gtest/gtest.h"
#include "solvers/qpmad_interface.hpp"

#ifdef POLYMPC_FOUND_QPMAD

TEST(QPMADSolverTest, qpmadSimpleQP)
{
    using Scalar = double;

    Eigen::Matrix<Scalar, 2,2> H;
    Eigen::Matrix<Scalar, 2,1> h;
    Eigen::Matrix<Scalar, 1,2> A;
    Eigen::Matrix<Scalar, 1,1> Al;
    Eigen::Matrix<Scalar, 1,1> Au;
    Eigen::Matrix<Scalar, 2,1> xl, xu, solution;

    H << 4, 1,
         1, 2;
    h << 1, 1;
    A << 1, 1;
    Al << 1;
    Au << 1;
    xl << 0, 0;
    xu << 0.7, 0.7;
    solution << 0.3, 0.7;

    polympc::QPMAD<2, 1, Scalar> prob;
    prob.settings().max_iter = 150;

    prob.solve(H,h,A,Al,Au,xl,xu);
    Eigen::Vector2d sol = prob.primal_solution();

    //EXPECT_TRUE(sol.isApprox(solution, 1e-2));
    EXPECT_EQ(prob.info().status, status_t::SOLVED);
}

#endif
