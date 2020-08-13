/*
 * grid_map_cv.hpp
 *
 *  Created on: Apr 14, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_cv/GridMapCvProcessing.hpp>
//#include <Eigen/src/Core/Matrix.h>

//void replaceNan(grid_map::Matrix& m, const double newValue)
//{
//    for(int r = 0; r < m.rows(); r++)
//    {
//        for(int c = 0; c < m.cols(); c++)
//        {
//            if (std::isnan(m(r,c)))
//            {
//                m(r,c) = newValue;
//            }
//        }
//    }
//}