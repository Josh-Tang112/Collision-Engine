#pragma once
#include <SFML/Graphics.hpp>

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>

#include <vector>

#include "../helper/readFile.hpp"
#include "../helper/STRtree.hpp"

// bounce off matrix:
// = exp(i(phi)) o reflect over x-axis o exp(i(-phi))
// = [cos(2 * phi) sin(2 * phi)]
//   [sin(2 * phi) -cos(2 * phi)]
// where phi is the degree between the surface and x-axis

// collision detection:
// course : STR Tree, cross product
// precise: dot product and projection

// orientation of points
// upper left -> upper right -> lower right -> lower left

struct Edge {
    int points[2][2]; // [0] - left point, [1] - right point
    float mat[2][2]; // bounce matrix
};

struct PhysicsSolver{
    std::vector<sf::Vector2f> pos, vel; // positions and velocity of balls
    struct STRTREE<struct Edge, 12> surface_tree;
    // std::vector<std::vector<std::vector<int>>> grid; // optimization, essentially a hash map
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

        for(int i = 0, e_count = 0; i < in.num_surface; i++){
            for(int j = 0; j < in.surf[i][0]; j++, e_count++){
                int p1x = ((int (*)[2])(in.surf[i] + 1))[j][0];
                int p1y = ((int (*)[2])(in.surf[i] + 1))[j][1];
                int p2x = ((int (*)[2])(in.surf[i] + 1))[(j + 1) % in.surf[i][0]][0];
                int p2y = ((int (*)[2])(in.surf[i] + 1))[(j + 1) % in.surf[i][0]][1];
                int bb[2][2], points[2][2] = {{p1x,p1y},{p2x,p2y}};
                get_bounding_box(points,bb);
                if(p1x >= p2x){
                    // swap p1, p2 to comform to the definition of points in Edge
                    int tmp;
                    tmp = p1x;
                    p1x = p2x;
                    p2x = tmp;

                    tmp = p1y;
                    p1y = p2y;
                    p2y = tmp;
                }
                float phi = atan2(p2y - p1y, p2x - p1x);
                float sin2 = std::sin(2 * phi);
                float cos2 = std::cos(2 * phi);
                // setting values
                edges[e_count] = {{{p1x,p1y}, {p2x,p2y}},{{cos2, sin2},{sin2, -1 * cos2}}};
                std::memcpy(bbs + e_count * 4, bb, 4 * sizeof(int));
            }
        }
        // adding the 4 edges of the window to the data structures appropriately
        int window_up[2][2] = {{0,0},{in.width,0}}, window_left[2] = {{0,0},{0, in.height}}, window_down[2][2] = {{0, in.height}, {in.width, in.height}}, window_right[2][2] = {{in.width,0},{in.width, in.height}};
        std::memcpy(bbs + num_edges - 4, window_up, 4 * sizeof(int));
        edges[num_edges - 4] = {{{0,0},{in.width,0}}, {{1, 0},{0,-1}}};
        std::memcpy(bbs + num_edges - 3, window_left, 4 * sizeof(int));
        edges[num_edges - 3] = {{{0,0},{in.width,0}}, {{-1, 0},{0,1}}};
        std::memcpy(bbs + num_edges - 2, window_down, 4 * sizeof(int));
        edges[num_edges - 2] = {{{0,0},{in.width,0}}, {{1, 0},{0,-1}}};
        std::memcpy(bbs + num_edges - 1, window_right, 4* sizeof(int));
        edges[num_edges - 1] = {{{0,0},{in.width,0}}, {{-1, 0},{0,1}}};

        this->surface_tree = STRTREE(bbs, edges, num_edges);

        for(int i = 0; i < in.num_ball; i ++){
            this->pos.push_back(sf::Vector2f(((int (*)[2])in.pos)[i][0],((int (*)[2])in.pos)[i][1]));
            this->vel.push_back(sf::Vector2f(((int (*)[2])in.vel)[i][0],((int (*)[2])in.vel)[i][1]));
        }

        std::free(bbs);
        std::free(edges);
    }

    // give the bounding box of any two points
    void get_bounding_box(int points[2][2], int bb[2][2]){
        if(points[0][0] < points[1][0]){
            bb[0][0] = points[0][0];
            bb[1][0] = points[1][0];
        }
        else{
            bb[0][0] = points[0][0];
            bb[1][0] = points[1][0];
        }
        if(points[0][1] < points[1][1]){
            bb[0][1] = points[0][1];
            bb[1][1] = points[1][1];
        }
        else{
            bb[0][1] = points[1][1];
            bb[1][1] = points[0][1];
        }
    }

    // calculate the distance between 2 points
    int point_point_distance(int p1[2], int p2[2]){
        int xdiff = p1[0] - p2[0], ydiff = p1[1] - p2[1];
        return xdiff * xdiff + ydiff * ydiff;
    }

    // calculate shortest squared distance from C to AB and find the closest point
    float point_line_dist(int A[2], int B[2], int C[2], float closest[2]){
        int CA[2] = {C[0] - A[0], C[1] - A[1]}, AB = {B[0] - A[0], B[1] - A[1]};
        float scalar = ((float)(CA[0] * AB[0] + CA[1] * AB[1])) / (AB[0] * AB[0] + AB[1] * AB[1]);
        if(scalar > 0 && scalar < 1){
            closest[0] = AB[0] * scalar;
            closest[1] = AB[1] * scalar;
            float p[2] = {C[0] - closest[0], C[1] - closest[1]};
            return p[0] * p[0] + p[1] * p[1];
        }
        else if(scalar <= 0){
            std::memcpy(closest,A,2 * sizeof(int));
            float p[2] = {C[0] - A[0], C[1] - A[1]};
        }
    }

    // whether AB intersect CD is equivalent to whether ACBD is a convex shape 
    // which can be tested by testing whether (CD X CA) and (CD X CB), (AB X AC) and (AB X AD) has different signs
    // to account for AB intersecting at C or D, cross product being zero will be counted as intersecting
    // we only need to compute the k component, which is just a determinant
    bool intersect(int A[2], int B[2], int C[2], int D[2]){
        int CD[2] = {D[0] - C[0], D[1] - C[1]}, CA[2] = {A[0] - C[0], A[1] - C[1]}, CB[2] = {B[0] - C[0], B[1] - C[1]};
        int CDXCA = CD[0] * CA[1] - CD[1] * CA[0], CDXCB = CD[0] * CB[1] - CD[1] * CB[0];
        if(CDXCA * CDXCB > 0){return false;}
        else if(CDXCA * CDXCB == 0){return true;}

        int AB[2] = {B[0] - A[0], B[1] - A[1]}, AC[2] = {C[0] - A[0], C[1] - A[1]}, AD[2] = {D[0] - A[0], D[1] - A[1]};
        int ABXAC = AB[0] * AC[1] - AB[1] * AC[0], ABXAD = AB[0] * AD[1] - AB[1] * AD[0];
        if(ABXAC * ABXAD > 0){return false;}
        else{return true;}
    }

    void posUpdate(sf::Vector2f accel, float collision_damping){
        sf::Vector2f tmp;
        for(unsigned int i = 0; i < this->pos.size(); i++){
            std::vector<struct Branch<struct Edge> *> *query_res;
            int q[2][2];
            
            // update velocity
            this->vel[i] += accel * this->delta_time;
            
            // preparing query
            sf::Vector2f old_pos = this->pos[i], new_pos = this->pos[i] + this->vel[i] * this->delta_time;
            int points[2][2] = {{old_pos.x, old_pos.y}, {new_pos.x,new_pos.y}}, bb[2][2];
            get_bounding_box(points, bb);

            // get query result
            query_res = this->surface_tree.query(bb);

            struct Edge e;
            for(auto b : *query_res){
                e = b->data;
                ;
            }
            
            if(min_dist == (unsigned int)-1){continue;}
            float vx = this->vel[i].x, vy = this->vel[i].y;
            this->vel[i].x = collision_damping * (vx * ((float (*)[2])matrix)[0][0] + vy * ((float (*)[2])matrix)[0][1]);
            this->vel[i].y = collision_damping * (vx * ((float (*)[2])matrix)[1][0] + vy * ((float (*)[2])matrix)[1][1]);
            
            delete query_res;
        }
    }

    void destroy(){
        this->surface_tree.destroy();
    }

};