// Markus Buchholz
#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <math.h>
#include <random>

//--------Path Planner--------------------------------------------------------------

float xmin = -5.0; // 0.0;
float xmax = 5.0;  // 50.0;
float ymin = -5.0; // 0.0;
float ymax = 5.0;  // 50.0;

float obsX = 25.0;
float obsY = 25.0;
float obsR = 5.0;

float goalX = 45.0;
float goalY = 45.0;

float startX = 2.0;
float startY = 2.0;

//--------------------------------------------------------------------------------
int DIM = 2;
int EVOLUTIONS = 500;
int INDIVIDUALS = 1000;
float M = 0.3;
float CR = 0.6;

//--------------------------------------------------------------------------------

struct Pos
{

    float x;
    float y;
};

//--------------------------------------------------------------------------------

float euclid(Pos a, Pos b)
{

    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}
//--------------------------------------------------------------------------------

float generateRandom()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_real_distribution<float> distribution(0.0, 1.0);
    return distribution(gen);
}

//--------------------------------------------------------------------------------

float generateRandomX()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_real_distribution<float> distribution(-1.0, 1.0);
    return distribution(gen);
}
//--------------------------------------------------------------------------------

float valueGenerator(float low, float high)
{

    return low + generateRandom() * (high - low);
}

//--------------------------------------------------------------------------------

std::vector<float> function(std::vector<Pos> pos)
{
    std::vector<float> funcValue;

    for (auto &ii : pos)
    {

        funcValue.push_back(ii.x * ii.y);
    }

    return funcValue;
}

//--------------------------------------------------------------------------------

float func(Pos pos)
{

    return pos.x * pos.y;
}

//--------------------------------------------------------------------------------

Pos positionUpdateCheck(Pos actualPos)
{

    Pos Pnew = actualPos;

    if (Pnew.x < xmin)
    {
        Pnew.x = xmin;
    }

    if (Pnew.x > xmax)
    {
        Pnew.x = xmax;
    }

    if (Pnew.y < ymin)
    {
        Pnew.y = ymin;
    }

    if (Pnew.y > ymax)
    {
        Pnew.y = ymax;
    }

    return Pnew;
}
//--------------------------------------------------------------------------------
int dimensionToUpdate()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_int_distribution<int> distribution(0, DIM);
    return distribution(gen);
}

//--------------------------------------------------------------------------------

Pos posMutation(Pos inA, Pos inB, Pos inC)
{

    Pos Xnew;

    Xnew.x = inA.x + M * (inB.x - inC.x);
    Xnew.x = inA.y + M * (inB.y - inC.y);

    return positionUpdateCheck(Xnew);
}

//--------------------------------------------------------------------------------

Pos posCrossover(Pos mut, Pos old)
{

    Pos posX;
    float rnd = generateRandom();

    if (rnd < CR)
    {

        int cross = dimensionToUpdate();

        if (cross == 0)
        {

            posX.x = mut.x;
            posX.y = old.y;
        }

        if (cross == 1)
        {

            posX.x = old.x;
            posX.y = mut.y;
        }

        if (cross == 2)
        {

            posX.x = mut.x;
            posX.y = mut.y;
        }

        return posX; // retun mutated
    }

    return old; // old value
}

//--------------------------------------------------------------------------------

std::vector<Pos> initPosXY()
{

    std::vector<Pos> pos;

    for (int ii = 0; ii < INDIVIDUALS; ii++)
    {

        pos.push_back({valueGenerator(xmin, xmax), valueGenerator(ymin, ymax)});
    }

    return pos;
}

//-------------------------------------------------------------------------------

std::tuple<int, int, int> choose3Individuals()
{

    std::random_device engine;
    std::uniform_int_distribution<int> distribution(0, INDIVIDUALS);

    int r1 = -1;
    int r2 = -1;
    int r3 = -1;

    do
    {

        r1 = distribution(engine);
        r2 = distribution(engine);
        r3 = distribution(engine);

    } while (r1 == r2 && r1 == r3 && r2 == r3);

    return std::make_tuple(r1, r2, r3);
}

//-------------------------------------------------------------------------------
int choose1Individual(int actual)
{

    std::random_device engine;
    std::uniform_int_distribution<int> distribution(0, INDIVIDUALS);

    int r = -1;

    do
    {

        r = distribution(engine);

    } while (r == actual);

    return r;
}

//-------------------------------------------------------------------------------

void runDE()
{

    std::vector<Pos> currentPositions = initPosXY();
    std::vector<float> currentValueFunction = function(currentPositions);

    for (int jj = 0; jj < EVOLUTIONS; jj++)
    {

        for (int ii = 0; ii < INDIVIDUALS; ii++)
        {

            auto individuals = choose3Individuals();
            auto in1 = std::get<0>(individuals);
            auto in2 = std::get<1>(individuals);
            auto in3 = std::get<2>(individuals);
            Pos mutatedPos = posMutation(currentPositions[in1], currentPositions[in2], currentPositions[in3]);

            int pos = choose1Individual(jj);
            Pos newCross = posCrossover(mutatedPos, currentPositions[pos]);
            auto newValueFunc = func(newCross);

            if (newValueFunc < currentValueFunction[ii])
            {

                currentValueFunction[ii] = newValueFunc;
                currentPositions[ii] = newCross;
            }
        }
    }

    for (auto &ii : currentValueFunction)
    {
        std::cout << ii << "\n";
    }
}

//-------------------------------------------------------------------------------

int main()
{

    runDE();
}
