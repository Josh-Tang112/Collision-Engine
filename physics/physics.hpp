#pragma once
#include <SFML/Graphics.hpp>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>

#include <vector>
#include <array>

#include "../helper/readFile.hpp"
#include "../helper/STRtree.hpp"

// bounce off matrix:
// = exp(i(phi)) o vertical reflection o exp(i(-phi))
// = [cos(2 * phi) sin(2 * phi)]
//   [sin(2 * phi) -cos(2 * phi)]
// where phi is the degree between the surface and x-axis

// collision detection:
// course : STR Tree, 
// precise: || x - (u * x)/(u * u) u || <= r? where x is the position vector and u is the surface vector

// orientation of points
// upper left -> upper right -> lower right -> lower left

struct Edge {
    int points[2][2]; // [0] - left point, [1] - right point
    float mat[2][2]; // bounce matrix
};

struct PhysicsSolver{
    std::vector<sf::Vector2f> pos, vel; // positions and velocity of balls
    struct STRTREE<std::array<std::array<float,2>,2>, 12> surface_tree;
    std::vector<std::vector<std::vector<int>>> grid; // optimization, essentially a hash map
    int cell_dim;
    float delta_time; 

    PhysicsSolver(struct FileReader in, int r, float delta_time) : 
        delta_time{delta_time}, cell_dim{2 * r} {
        // calculate the dim of surface
        int num_edges = 0;
        for(int i = 0; i < in.num_surface; i++){
            num_edges += in.surf[0];
        }
        num_edges += 4; // for the 4 edges of the window

        struct Edge *edges = (struct Edge*)std::malloc(num_edges * sizeof(struct Edge));
        int *bbs = (int*)std::malloc(num_edges * 4 * sizeof(int)); // 4 int for the bounding box

        for(int i = 0; i < in.num_surface; i++){
            for(int j = 0; j < in.surf[i][0]; j++){
                int p1x = ((int (*)[2])(in.surf[i] + 1))[j][0];
                int p1y = ((int (*)[2])(in.surf[i] + 1))[j][1];
                int p2x = ((int (*)[2])(in.surf[i] + 1))[(j + 1) % in.surf[i][0]][0];
                int p2y = ((int (*)[2])(in.surf[i] + 1))[(j + 1) % in.surf[i][0]][1];
                float phi = atan2(p2y - p1y, p2x - p1x);
                float sin2 = std::sin(2 * phi);
                float cos2 = std::cos(2 * phi);
                struct Edge e = {{p1x,p1y}, {p2x,p2y},{{cos2, sin2},{sin2, -1 * cos2}}};
            }
        }

        this->surface_tree = STRTREE(bbs, edges, num_edges);

        for(int i = 0; i < in.num_ball; i ++){
            this->pos.push_back(sf::Vector2f(((int (*)[2])in.pos)[i][0],((int (*)[2])in.pos)[i][1]));
            this->vel.push_back(sf::Vector2f(((int (*)[2])in.vel)[i][0],((int (*)[2])in.vel)[i][1]));
        }

        std::free(bbs);
        std::free(edges);
    }

    void posUpdate(sf::Vector2f accel, float collision_damping){
        for(unsigned int i = 0; i < this->pos.size(); i++){
            this->vel[i] += accel * this->delta_time;
            this->pos[i] += this->vel[i] * this->delta_time;
            int px = this->pos[i].x / this->cell_dim, py = this->pos[i].y / this->cell_dim, px_p = px + 1, py_p = py + 1;
            px = (px)? px - 1: px;
            py = (py)? py - 1: py;
            float min_dist = (unsigned int)-1, **matrix;
            for(int i = px; i < px_p + 1; i++){
                for(int j = py; j < py_p + 1; j++){
                    if(!this->surfaces[j + 1][i + 1].size()){continue;}

                    // for(auto e : this->surfaces[j + 1][i + 1]){
                    //     float u[2] = {e.p2[0] - e.p1[0], e.p2[1] - e.p1[1]};
                    //     float x[2] = {this->pos[i].x, this->pos[i].y};
                    //     float ux = u[0] * x[0] + u[1] * x[1];
                    //     int uu = u[0] * u[0] + u[1] * u[1];
                    //     float scaler = ((float)ux) / uu;
                    //     float dist[2] = {x[0] - scaler * u[0], x[1] - scaler * u[1]};
                    //     float distance = dist[0] * dist[0] + dist[1] * dist[1];
                    //     if(distance < min_dist && distance <= (this->cell_dim * this->cell_dim / 4)){
                    //         min_dist = distance;
                    //         matrix = (float **)e.mat;
                    //     }
                    // }

                    for(auto e : this->surfaces[j + 1][i + 1]){
                        float u[2] = {(e.p2[0] + e.p1[0]) / 2, (e.p2[1] + e.p1[1]) / 2};
                        float x[2] = {this->pos[i].x, this->pos[i].y};
                        float dist[2] = {x[0] - u[0], x[1] - u[1]};
                        float distance = dist[0] * dist[0] + dist[1] * dist[1];
                        if(distance < min_dist && distance <= (this->cell_dim * this->cell_dim / 4)){
                            min_dist = distance;
                            matrix = (float **)e.mat;
                        }
                    }
                }
            } 
            if(min_dist == (unsigned int)-1){continue;}
            float vx = this->vel[i].x, vy = this->vel[i].y;
            this->vel[i].x = collision_damping * (vx * ((float (*)[2])matrix)[0][0] + vy * ((float (*)[2])matrix)[0][1]);
            this->vel[i].y = collision_damping * (vx * ((float (*)[2])matrix)[1][0] + vy * ((float (*)[2])matrix)[1][1]);
        }
    }

    void destroy(){
        this->surface_tree.destroy();
    }

};