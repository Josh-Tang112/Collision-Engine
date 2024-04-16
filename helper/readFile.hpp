#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>

struct FileReader {
    int width, height, num_ball, num_surface, *num_points_per_surface;
    float *pos, *vel, **surf;

    FileReader(char fname[]){
        FILE *in = std::fopen(fname, "r");
        char buff[1024], *notNULL, *tok;
        int win[2], c = 0;
        notNULL = std::fgets(buff, 1024, in);
        assert(notNULL);

        tok = std::strtok(buff, " "); 
        while(tok){
            win[c] = std::atoi(tok);
            c += 1;
            tok = std::strtok(NULL, " ");
        }
        this->width = win[0];
        this->height = win[1]; 

        notNULL = std::fgets(buff, 1024, in);
        assert(notNULL);
        this->num_ball = std::atoi(buff);
        this->pos = (float *)std::malloc(2 * sizeof(float) * this->num_ball);
        this->vel = (float *)std::malloc(2 * sizeof(float) * this->num_ball);
        for(int i = 0; i < this->num_ball; i++){
            int tmp = 0;
            notNULL = std::fgets(buff, 1024, in);
            assert(notNULL);
            tok = std::strtok(buff, " ");
            while(tok){
                switch(tmp){
                    case 0:
                    case 1:
                        ((float (*)[2])this->pos)[i][tmp % 2] = std::atof(tok); break;
                    case 2:
                    case 3:
                        ((float (*)[2])this->vel)[i][tmp % 2] = std::atof(tok); break;
                }
                tmp += 1;
                tok = std::strtok(NULL, " ");
            }
        }

        notNULL = std::fgets(buff, 1024, in);
        assert(notNULL);
        this->num_surface = std::atoi(buff);
        this->num_points_per_surface = (int*)std::malloc(sizeof(int) * this->num_surface);
        this->surf = (float **)std::malloc(sizeof(float *) * this->num_surface);
        for(int i = 0; i < this->num_surface; i++){
            int pcount = 0;
            notNULL = std::fgets(buff, 1024, in);
            assert(notNULL);
            tok = std::strtok(buff, " ");
            pcount = std::atoi(tok);
            assert(pcount > 1); // assert each line should have at least 2 points to form a surface
            this->num_points_per_surface[i] = pcount;
            this->surf[i] = (float *)std::malloc(sizeof(float) * 2 * pcount);
            
            tok = std::strtok(NULL, " ");
            pcount = 0;
            while(tok){
                this->surf[i][pcount] = std::atof(tok);
                pcount += 1;
                tok = std::strtok(NULL, " ");
                assert(pcount <= 2 * this->num_points_per_surface[i]);
            }
        }

        std::fclose(in);
    } 

    void Free(){
        std::free(this->pos);
        std::free(this->vel);
        std::free(this->num_points_per_surface);
        for(int i = 0; i < this->num_surface; i++){
            std::free(this->surf[i]);
        }
        std::free(this->surf);
    }
};