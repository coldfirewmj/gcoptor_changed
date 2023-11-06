/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef VOXEL_MAP_HPP
#define VOXEL_MAP_HPP

#include "voxel_dilater.hpp"
#include <memory>
#include <vector>
#include <Eigen/Eigen>

namespace voxel_map
{

    constexpr uint8_t Unoccupied = 0;
    constexpr uint8_t Occupied = 1;
    constexpr uint8_t Dilated = 2;

    class VoxelMap
    {

    public:
        VoxelMap() = default;
        VoxelMap(const Eigen::Vector3i &size,
                 const Eigen::Vector3d &origin,
                 const double &voxScale)
            : mapSize(size),
              o(origin),
              scale(voxScale),
              voxNum(mapSize.prod()),
              step(1, mapSize(0), mapSize(1) * mapSize(0)),
              oc(o + Eigen::Vector3d::Constant(0.5 * scale)),
              bounds((mapSize.array() - 1) * step.array()),
              stepScale(step.cast<double>().cwiseInverse() * scale),
              voxels(voxNum, Unoccupied) {
                esdf_list.resize(voxNum,0.0);
                get_influence_vox(voxScale,voxScale);
                std::cout<<"origin is"<<o.transpose()<<std::endl;
                std::cout<<"bounds is"<<bounds.transpose()<<std::endl;
              }

    private:
        Eigen::Vector3i mapSize;
        Eigen::Vector3d o;
        double scale;
        int voxNum;
        Eigen::Vector3i step;
        Eigen::Vector3d oc;
        Eigen::Vector3i bounds;
        Eigen::Vector3d stepScale;
        std::vector<uint8_t> voxels;
        double sign_dis;
        std::vector<Eigen::Vector3i> influence_vox;
        std::vector<Eigen::Vector3i> surf;
    public:
        std::vector<double> esdf_list;
        inline Eigen::Vector3i getSize(void) const
        {
            return mapSize;
        }

        inline double getScale(void) const
        {
            return scale;
        }

        inline Eigen::Vector3d getOrigin(void) const
        {
            return o;
        }

        inline Eigen::Vector3d getCorner(void) const
        {
            return mapSize.cast<double>() * scale + o;
        }

        inline void get_influence_vox(double resulotion,double inflate)
        {
            sign_dis=inflate*6;
            int mesh_idx=(int)ceil(5*inflate/resulotion);
            for (int i=-mesh_idx;i<mesh_idx+1;i++)
            {
                for (int j=-mesh_idx;j<mesh_idx+1;j++)
                {
                    for (int k=-mesh_idx;k<mesh_idx+1;k++)
                    {
                        Eigen::Vector3i input;
                        double e_dis=sqrt(i*i+j*j+k*k);
                        if (e_dis*resulotion<sign_dis)
                        {
                            input.x()=i;input.y()=j;input.z()=k;
                            influence_vox.push_back(input);
                        }
                    }
                }
            }
            std::cout<<"influence_vox size is: "<<influence_vox.size()<<std::endl;
        }

        inline const std::vector<uint8_t> &getVoxels(void) const
        {
            return voxels;
        }

        inline void update_esdf()
        {
            for (int t=0;t<surf.size();t++)
            {
                Eigen::Vector3i input(surf[t](0),surf[t](1)/step(1),surf[t](2)/step(2));
                for (int i=0;i<influence_vox.size();i++)
                {
                    Eigen::Vector3i loc=input+influence_vox[i];
                    if (loc(0) >= 0 && loc(1) >= 0 && loc(2) >= 0 &&
                    loc(0) < mapSize(0) && loc(1) < mapSize(1) && loc(2) < mapSize(2))
                    {
                        int idx=loc.dot(step);
                        double val=sign_dis-scale*(loc-input).norm();
                        esdf_list[idx]=(esdf_list[idx]<val)?val:esdf_list[idx];
                        //voxels[idx] = Occupied;
                    }
                }
            }
            std::cout<<"max esdflist is:"<<*max_element(esdf_list.begin(),esdf_list.end())<<std::endl;
        }

        inline double get_esdf(Eigen::Vector3i input)
        {
            return esdf_list[input.dot(step)];
        }

        inline void setOccupied(const Eigen::Vector3d &pos)
        {
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxels[id.dot(step)] = Occupied;
            }
        }

        inline void setOccupied(const Eigen::Vector3i &id)
        {
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxels[id.dot(step)] = Occupied;
            }
        }

        //该函数用来扩大障碍物区域
        inline void dilate(const int &r)
        {
            if (r <= 0)
            {
                return;
            }
            else
            {
                std::vector<Eigen::Vector3i> lvec, cvec;
                lvec.reserve(voxNum);
                cvec.reserve(voxNum);
                int i, j, k, idx;
                bool check;
                for (int x = 0; x <= bounds(0); x++)
                {
                    for (int y = 0; y <= bounds(1); y += step(1))
                    {
                        for (int z = 0; z <= bounds(2); z += step(2))
                        {
                            if (voxels[x + y + z] == Occupied)
                            {
                                VOXEL_DILATER(i, j, k,
                                              x, y, z,
                                              step(1), step(2),
                                              bounds(0), bounds(1), bounds(2),
                                              check, voxels, idx, Dilated, cvec)
                            }
                        }
                    }
                }

                for (int loop = 1; loop < r; loop++)
                {
                    std::swap(cvec, lvec);
                    for (const Eigen::Vector3i &id : lvec)
                    {
                        VOXEL_DILATER(i, j, k,
                                      id(0), id(1), id(2),
                                      step(1), step(2),
                                      bounds(0), bounds(1), bounds(2),
                                      check, voxels, idx, Dilated, cvec)
                    }
                    lvec.clear();
                }

                surf = cvec;
            }
        }

        inline void getSurfInBox(const Eigen::Vector3i &center,
                                 const int &halfWidth,
                                 std::vector<Eigen::Vector3d> &points) const
        {
            for (const Eigen::Vector3i &id : surf)
            {
                if (std::abs(id(0) - center(0)) <= halfWidth &&
                    std::abs(id(1) / step(1) - center(1)) <= halfWidth &&
                    std::abs(id(2) / step(2) - center(2)) <= halfWidth)
                {
                    points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
                }
            }

            return;
        }

        inline void getSurf(std::vector<Eigen::Vector3d> &points) const
        {
            //oc为原点+1/2*resolution
            points.reserve(surf.size());
            for (const Eigen::Vector3i &id : surf)
            {
                //cast<double>()为int转换为double，cwiseProduct为逐元素乘法
                points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
            }
            return;
        }

        inline bool query(const Eigen::Vector3d &pos) const
        {
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                return voxels[id.dot(step)];
            }
            else
            {
                return true;
            }
        }

        inline bool query(const Eigen::Vector3i &id) const
        {
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                return voxels[id.dot(step)];
            }
            else
            {
                return true;
            }
        }

        inline Eigen::Vector3d posI2D(const Eigen::Vector3i &id) const
        {
            return id.cast<double>() * scale + oc;
        }

        inline Eigen::Vector3i posD2I(const Eigen::Vector3d &pos) const
        {
            return ((pos - o) / scale).cast<int>();
        }

        inline std::vector<Eigen::Vector3d> new_surf(double new_voxScale) const
        {
            double coeff=new_voxScale/scale;
            Eigen::Vector3d new_o=o*coeff;
            Eigen::Vector3d new_oc=new_o + Eigen::Vector3d::Constant(0.5 * new_voxScale);
            Eigen::Vector3d new_stepScale=step.cast<double>().cwiseInverse() * new_voxScale;
            std::vector<Eigen::Vector3d> points;
            points.reserve(surf.size());
            for (const Eigen::Vector3i &id : surf)
            {
                //cast<double>()为int转换为double，cwiseProduct为逐元素乘法
                points.push_back(id.cast<double>().cwiseProduct(new_stepScale) + new_oc);
            }
            return points;
        }
    };
}

#endif
