#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>

struct FileReader {
    int width, height, num_ball, num_surface;
    int *pos, *vel, **surf;

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
        this->pos = (int *)std::malloc(2 * sizeof(int) * this->num_ball);
        this->vel = (int *)std::malloc(2 * sizeof(int) * this->num_ball);
        for(int i = 0; i < this->num_ball; i++){
            int tmp = 0;
            notNULL = std::fgets(buff, 1024, in);
            assert(notNULL);
            tok = std::strtok(buff, " ");
            while(tok){
                switch(tmp){
                    case 0:
                    case 1:
                        ((int (*)[2])this->pos)[i][tmp % 2] = std::atoi(tok); break;
                    case 2:
                    case 3:
                        ((int (*)[2])this->vel)[i][tmp % 2] = std::atoi(tok); break;
                }
                tmp += 1;
                tok = std::strtok(NULL, " ");
            }
        }

        notNULL = std::fgets(buff, 1024, in);
        assert(notNULL);
        this->num_surface = std::atoi(buff);
        this->surf = (int **)std::malloc(sizeof(int *) * this->num_surface);
        for(int i = 0; i < this->num_surface; i++){
            int pcount = 0;
            notNULL = std::fgets(buff, 1024, in);
            assert(notNULL);
            tok = std::strtok(buff, " ");
            pcount = std::atoi(tok);
            assert(pcount > 1); // assert each line should have at least 2 points to form a surface
            this->surf[i] = (int *)std::malloc(sizeof(int) * (2 * pcount + 1));
            this->surf[i][0] = pcount;
            tok = std::strtok(NULL, " ");
            pcount = 1;
            while(tok){
                this->surf[i][pcount] = std::atoi(tok);
                pcount += 1;
                tok = std::strtok(NULL, " ");
            }
            assert(pcount % 2 == 1); // assert that number of integers in a line is odd
        }

        std::fclose(in);
    } 

    void Free(){
        std::free(this->pos);
        std::free(this->vel);
        for(int i = 0; i < this->num_surface; i++){
            std::free(this->surf[i]);
        }
        std::free(this->surf);
    }
};