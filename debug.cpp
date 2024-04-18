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
    int count = 0;
    while(!sol.posUpdate(accel,0.8)){
        count++;
    }

    printf("%d\n",count);

    in.Free();
    sol.destroy();

    return 0;
}