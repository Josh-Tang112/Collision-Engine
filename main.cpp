#include <SFML/Graphics.hpp>
#include <cstdio>

#include "physics/physics.hpp"
#include "helper/readFile.hpp"

#define GRAVITY 10

int main(int argc, char *argv[])
{

    float delta_time = 1./40, collision_damping = 0.8, r = 10, d = 2* r;

    struct FileReader in = FileReader(argv[1]);
    struct PhysicsSolver sol = PhysicsSolver(in, r, delta_time);

    sf::RenderWindow window(sf::VideoMode(in.width,in.height), "Ball Collider");
    window.setSize(sf::Vector2u(in.width,in.height)); 

    sf::Vector2f accel(0, GRAVITY);
    sf::CircleShape cir_lst[in.num_ball];
    for(int i = 0; i < in.num_ball; i++){
        cir_lst[i] = sf::CircleShape(r);
        cir_lst[i].setFillColor(sf::Color::Red);
        cir_lst[i].setPosition(sol.pos[i].x, sol.pos[i].y);
    }
    sf::ConvexShape shape_lst[in.num_surface];
    for(int i = 0; i < in.num_surface; i++){
        shape_lst[i] = sf::ConvexShape(in.surf[i][0]);
        shape_lst[i].setFillColor(sf::Color::White);
        for(int j = 0; j < in.surf[i][0]; j++){
            shape_lst[i].setPoint(j,sf::Vector2f(((int (*)[2])(in.surf[i] + 1))[j][0], ((int (*)[2])(in.surf[i] + 1))[j][1]));
        }
    }

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed){
                window.close();
                in.Free();
                sol.destroy();
            }
        }

        window.clear();
        for(int i = 0; i < in.num_ball; i++){
            window.draw(cir_lst[i]);
        }
        for(int i = 0; i < in.num_surface; i++){
            window.draw(shape_lst[i]);
        }

        // update position
        sol.posUpdate(accel,collision_damping);
        for(int i = 0; i < in.num_ball; i++){
            cir_lst[i].setPosition(sol.pos[i].x, sol.pos[i].y);
        }

        window.display();
    }

    return 0;
}