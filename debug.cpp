#include <SFML/Graphics.hpp>
#include <cstdio>
#include <cstdlib>

#include "helper/readFile.hpp"
#include "helper/STRtree.hpp"
#include "physics/physics.hpp"

int main(int argc, char *argv[])
{
    struct FileReader in(argv[1]);
    struct PhysicsSolver sol(in, 10, 1./40);

    sf::Vector2f accel(0, 10);
    int step_count = 0, collision_count = 0;
    float vel = 1;
    while(step_count < 100000){
        if(sol.posUpdate(accel,0.8)){
            collision_count++;
        }
        vel = sol.vel[0].x * sol.vel[0].x + sol.vel[0].y * sol.vel[0].y;
        step_count++;
    }

    printf("%d %f\n",collision_count,vel);

    printf("pos: %f %f, vel: %f %f\n",sol.pos[0].x,sol.pos[0].y,sol.vel[0].x,sol.vel[0].y);
    // printf("%d\n",sol.posUpdate(accel,0.8));
    // printf("pos: %f %f, vel: %f %f\n",sol.pos[0].x,sol.pos[0].y,sol.vel[0].x,sol.vel[0].y);

    sf::RenderWindow window(sf::VideoMode(in.width, in.height), "My window");
    sf::CircleShape shape(10);
    shape.setFillColor(sf::Color::White);
    shape.setPosition(5,5);
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::Black);

        
        window.draw(shape);

        window.display();
    }

    in.Free();
    sol.destroy();

    return 0;
}