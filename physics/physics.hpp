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
    float points[2][2]; 
    float mat[2][2]; // bounce matrix
};

struct PhysicsSolver{
    std::vector<sf::Vector2f> pos, vel; // positions and velocity of balls
    struct STRTREE<struct Edge, 12> surface_tree;
    // std::vector<std::vector<std::vector<int>>> grid; // optimization, essentially a hash map
    int r;
    float delta_time; 

    PhysicsSolver(struct FileReader in, int r, float delta_time) : 
        delta_time{delta_time}, r{r} {
        // calculate the dim of surface
        int num_edges = 0;
        for(int i = 0; i < in.num_surface; i++){
            num_edges += in.num_points_per_surface[i];
        }
        num_edges += 4; // for the 4 edges of the window

        struct Edge *edges = (struct Edge*)std::malloc(num_edges * sizeof(struct Edge));
        float *bbs = (float*)std::malloc(num_edges * 4 * sizeof(float)); // 4 int for the bounding box

        for(int i = 0, e_count = 0; i < in.num_surface; i++){
            for(int j = 0; j < in.num_points_per_surface[i]; j++, e_count++){
                float p1x = ((float (*)[2])(in.surf[i]))[j][0];
                float p1y = ((float (*)[2])(in.surf[i]))[j][1];
                float p2x = ((float (*)[2])(in.surf[i]))[(j + 1) % in.num_points_per_surface[i]][0];
                float p2y = ((float (*)[2])(in.surf[i]))[(j + 1) % in.num_points_per_surface[i]][1];
                float bb[2][2], points[2][2] = {{p1x,p1y},{p2x,p2y}};
                get_bounding_box(points,bb);
                float phi = atan2(p2y - p1y, p2x - p1x);
                float sin2 = std::sin(2 * phi);
                float cos2 = std::cos(2 * phi);
                // setting values
                edges[e_count] = {{{p1x,p1y},{p2x,p2y}},{{cos2, sin2},{sin2, -1 * cos2}}};
                std::memcpy(bbs + e_count * 4, bb, 4 * sizeof(float));
            }
        }
        // adding the 4 edges of the window to the data structures appropriately
        float window_up[2][2] = {{0,0},{(float)in.width,0}}, window_left[2][2] = {{0,0},{0, (float)in.height}};
        float window_down[2][2] = {{0, (float)in.height}, {(float)in.width, (float)in.height}};
        float window_right[2][2] = {{(float)in.width,0},{(float)in.width, (float)in.height}};
        std::memcpy(bbs + (num_edges - 4) * 4, window_up, 4 * sizeof(float));
        edges[num_edges - 4] = {{{0,0},{(float)in.width,0}}, {{1, 0},{0,-1}}};
        std::memcpy(bbs + (num_edges - 3) * 4, window_left, 4 * sizeof(float));
        edges[num_edges - 3] = {{{0,0},{0, (float)in.height}}, {{-1, 0},{0,1}}};
        std::memcpy(bbs + (num_edges - 2) * 4, window_down, 4 * sizeof(float));
        edges[num_edges - 2] = {{{0, (float)in.height}, {(float)in.width, (float)in.height}}, {{1, 0},{0,-1}}};
        std::memcpy(bbs + (num_edges - 1) * 4, window_right, 4* sizeof(float));
        edges[num_edges - 1] = {{{(float)in.width,0},{(float)in.width, (float)in.height}}, {{-1, 0},{0,1}}};

        struct STRTREE<struct Edge, 12> tmp_tree((float***)bbs, edges, num_edges);
        this->surface_tree.root = tmp_tree.root;

        for(int i = 0; i < in.num_ball; i ++){
            this->pos.push_back(sf::Vector2f(((float (*)[2])in.pos)[i][0],((float (*)[2])in.pos)[i][1]));
            this->vel.push_back(sf::Vector2f(((float (*)[2])in.vel)[i][0],((float (*)[2])in.vel)[i][1]));
        }

        std::free(bbs);
        std::free(edges);
    }

    // give the bounding box of any two points
    void get_bounding_box(float points[2][2], float bb[2][2]){
        if(points[0][0] < points[1][0]){
            bb[0][0] = points[0][0];
            bb[1][0] = points[1][0];
        }
        else{
            bb[0][0] = points[1][0];
            bb[1][0] = points[0][0];
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
    double point_point_distance(float p1[2], float p2[2]){
        float xdiff = p1[0] - p2[0], ydiff = p1[1] - p2[1];
        return xdiff * xdiff + ydiff * ydiff;
    }

    // calculate shortest squared distance from C to AB and find the closest point
    double point_line_dist(float A[2], float B[2], float C[2], float closest[2]){
        float CA[2] = {C[0] - A[0], C[1] - A[1]}, AB[2] = {B[0] - A[0], B[1] - A[1]};
        float scalar = (CA[0] * AB[0] + CA[1] * AB[1]) / (AB[0] * AB[0] + AB[1] * AB[1]);
        if(scalar > 0 && scalar < 1){
            closest[0] = AB[0] * scalar;
            closest[1] = AB[1] * scalar;
        }
        else if(scalar <= 0){
            std::memcpy(closest,A,2 * sizeof(float));
        }
        else {
            std::memcpy(closest,B,2 * sizeof(float));
        }
        float p[2] = {C[0] - closest[0], C[1] - closest[1]};
        return p[0] * p[0] + p[1] * p[1];
    }

    // whether AB intersect CD is equivalent to whether ACBD is a convex shape 
    // which can be tested by testing whether (CD X CA) and (CD X CB), (AB X AC) and (AB X AD) has different signs
    // to account for AB intersecting at C or D, cross product being zero will be counted as intersecting
    // we only need to compute the k component, which is just a determinant
    bool intersect(float A[2], float B[2], float C[2], float D[2]){
        float CD[2] = {D[0] - C[0], D[1] - C[1]}, CA[2] = {A[0] - C[0], A[1] - C[1]}, CB[2] = {B[0] - C[0], B[1] - C[1]};
        float CDXCA = CD[0] * CA[1] - CD[1] * CA[0], CDXCB = CD[0] * CB[1] - CD[1] * CB[0];
        if(CDXCA * CDXCB > 0){return false;}
        else if(CDXCA * CDXCB == 0){return true;}

        float AB[2] = {B[0] - A[0], B[1] - A[1]}, AC[2] = {C[0] - A[0], C[1] - A[1]}, AD[2] = {D[0] - A[0], D[1] - A[1]};
        float ABXAC = AB[0] * AC[1] - AB[1] * AC[0], ABXAD = AB[0] * AD[1] - AB[1] * AD[0];
        if(ABXAC * ABXAD > 0){return false;}
        else{return true;}
    }

    // find the intersection of two lines AB and VW, and return whether the intersection is on line segments AB and VW
    // we find the intersection using parameterization and determinant
    bool intersection(float A[2], float B[2], float V[2], float W[2], float intersection[2]){
        float s,t;
        float a = W[0] - V[0], b = A[0] - B[0], c = W[1] - V[1], d = A[1] - B[1], e = A[0] - V[0], f = A[1] - V[1];
        float det = a * d - b * c;
        if(!det){return false;}
        s = (e * d - b * f) / det;
        t = (a * f - c * e) / det;
        intersection[0] = A[0] - t * b;
        intersection[1] = A[1] - t * d;

        return (s >= 0 && s <= 1) && (t >= 0 && t <= 1);
    }

    void posUpdate(sf::Vector2f accel, float collision_damping){
        for(unsigned int i = 0; i < this->pos.size(); i++){
            std::vector<struct Branch<struct Edge> *> *query_res;
            
            // update velocity
            this->vel[i] += accel * this->delta_time;

            // calculate a vector parallel to the velocity vector with size r
            float vel_size = std::sqrt(this->vel[i].x * this->vel[i].x + this->vel[i].y * this->vel[i].y); 
            float direction[2] = {this->vel[i].x * this->r / vel_size, this->vel[i].y * this->r / vel_size};
            
            // preparing query
            sf::Vector2f old_pos = this->pos[i], new_pos = this->pos[i] + this->vel[i] * this->delta_time;
            float points[2][2] = {{old_pos.x, old_pos.y}, {new_pos.x,new_pos.y}};

            float bb[2][2];
            get_bounding_box(points, bb);

            // get query result
            query_res = this->surface_tree.query(bb);

            struct Edge e;
            float intersect[2];
            this->pos[i] += this->vel[i] * this->delta_time;
            for(auto b : *query_res){
                e = b->data;
                if(intersection(points[0], points[1], (float*)e.points[0], (float*)e.points[1], intersect)){
                    this->pos[i] = sf::Vector2f(intersect[0] - direction[0], intersect[1] - direction[1]);
                    float vx = this->vel[i].x, vy = this->vel[i].y;
                    this->vel[i].x = collision_damping * (vx * e.mat[0][0] + vy * e.mat[0][1]);
                    this->vel[i].y = collision_damping * (vx * e.mat[1][0] + vy * e.mat[1][1]);
                    break;
                }
            }
            
            delete query_res;
        }
    }

    void destroy(){
        this->surface_tree.destroy();
    }

};