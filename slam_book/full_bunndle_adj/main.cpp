#include "full_bundle_adj_problem.h"

int main(int argc, char *argv[])
{
    full_bla_problem fbla;
    fbla.load("../data/problem-16-22106-pre.txt");
    fbla.write_ply("before.txt");
    fbla.build_up();
    fbla.solve();
    fbla.update();
    fbla.write_ply("optimized.txt");
//    fbla.cam_rt_test();
    return 0;
}
