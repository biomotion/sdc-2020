#include <iostream>
#include <cstdlib>

using namespace std;

#define SUNNY 0
#define CLOUDY 1
#define RAINY 2

uint weather_tomorrow(uint);
void find_distrib(void);

int main(){

    const int n_days = 100, n_sims = 10;
    uint first_day;
    uint *weathers = new uint[n_days];
    srand(time(NULL));


    find_distrib();
    // cout << "What's the weather of Day1? (0:Sunny, 1:Cloudy, 2:Rainy)" << endl << ">>> ";
    // cin >> first_day;

    // cout << "Generating " << n_sims << " sequences of " << n_days << " days of simulations..." << endl
    //      << "0=Sunny, 1=Cloudy, 2=Rainy" <<  endl;

    // for(uint i=0; i<n_sims; i++){
    //     weathers[0] = first_day;
    //     cout << "Sim " << i << ": ";
    //     for(uint j=0; j<n_days; j++){
    //         cout << weathers[j] << ", ";  
    //         weathers[j+1] = weather_tomorrow(weathers[j]);
    //     }
    //     cout << endl;
    // }
    return 0;

}

uint weather_tomorrow(uint today){
    double transision[3][3] = {
        {.8, .2, 0},
        {.4, .4, .2},
        {.2, .6, .2}
    };
    
    uint x = rand()%100;
    if(x < transision[today][0]*100) return SUNNY;
    else if (x < transision[today][0]*100 + transision[today][1]*100) return CLOUDY;
    else return RAINY;
}

void find_distrib(void){
// Run N times of simulation finding the weather of a random day
// Accumulate the result to find the distribution
    uint first_day=0, next_day;
    const uint N = 100000;
    uint times_sunny=0, times_cloudy=0, times_rainy=0;
    for(uint i=0; i<N; i++){
        next_day = weather_tomorrow(first_day);
        for(uint j=0; j<rand()%100000; j++){
            next_day = weather_tomorrow(next_day);
        }
        if(next_day == SUNNY) times_sunny++;
        else if(next_day == CLOUDY) times_cloudy++;
        else if(next_day == RAINY) times_rainy++;
    }
    cout << "Distribution: " << "sunny: " << (double)times_sunny/N << ", cloudy: " << (double)times_cloudy/N << ", rainy: " << (double)times_rainy/N << endl;
    return;

}