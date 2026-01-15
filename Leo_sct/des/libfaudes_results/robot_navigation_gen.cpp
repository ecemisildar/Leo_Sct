// workspace/robot_navigation.cpp

#include "libfaudes.h"
#include "syn_supcon.h"   // supervisor synthesis
#include <vector>
#include <string>
#include <iostream>

using namespace faudes;

// List of controllable & uncontrollable event names
static const std::vector<std::string> kControllableEvents = {
    "move_forward",
    "move_backward",
    "rotate_clockwise",
    "rotate_counterclockwise",
    "full_rotate",
    "random_walk",
    "slow_down",
    "speed_up"
};

static const std::vector<std::string> kUncontrollableEvents = {
    "path_clear",
    "obstacle_front",
    "obstacle_left",
    "obstacle_right",
    "crowd_detected"
};

// Helper: add all events to a generator (no attributes, just names)
void InsertEvents(Generator& G) {
    for (const auto& e : kControllableEvents) {
        G.InsEvent(e);
    }
    for (const auto& e : kUncontrollableEvents) {
        G.InsEvent(e);
    }
}

// Build a free-behaviour plant model: everything physically possible
Generator MakeFreeRobotModel() {

    Generator G;
    G.Name("G_robot_free");

    InsertEvents(G);

    // Single state with all events as self-loops
    Idx q = G.InsState("q");
    G.SetInitState(q);
    G.SetMarkedState(q);

    for (EventSet::Iterator it = G.AlphabetBegin(); it != G.AlphabetEnd(); ++it) {
        Idx e = *it;
        G.SetTransition(q, e, q);
    }

    return G;
}

// Build a specification generator for reactive motion
Generator MakeSpecReactiveMotion() {

    Generator K;
    K.Name("K_reactive_motion");

    InsertEvents(K);

    // States (sensing modes)
    K.InsState("clear");
    K.InsState("obs_left");
    K.InsState("obs_right");
    K.InsState("obs_front");
    K.InsState("stuck");
    K.InsState("crowded");

    K.SetInitState("clear");
    K.SetMarkedState("clear");
    K.SetMarkedState("obs_left");
    K.SetMarkedState("obs_right");
    K.SetMarkedState("obs_front");
    K.SetMarkedState("stuck");
    K.SetMarkedState("crowded");

    // --- Sensor transitions (mode switching) ---
K.SetTransition("clear", "move_forward", "clear");
K.SetTransition("clear", "random_walk", "clear");
K.SetTransition("clear", "rotate_clockwise", "clear");
K.SetTransition("clear", "rotate_counterclockwise", "clear");
K.SetTransition("clear", "speed_up", "clear");
K.SetTransition("clear", "slow_down", "clear");
K.SetTransition("clear", "obstacle_front", "obs_front");
K.SetTransition("clear", "obstacle_left", "obs_left");
K.SetTransition("clear", "obstacle_right", "obs_right");
K.SetTransition("clear", "crowd_detected", "crowded");
K.SetTransition("clear", "path_clear", "clear");
K.SetTransition("obs_front", "rotate_clockwise", "obs_right");
K.SetTransition("obs_front", "rotate_counterclockwise", "obs_left");
K.SetTransition("obs_front", "full_rotate", "clear");
K.SetTransition("obs_front", "random_walk", "obs_front");
K.SetTransition("obs_front", "crowd_detected", "crowded");
K.SetTransition("obs_front", "path_clear", "clear");
K.SetTransition("obs_front", "obstacle_left", "obs_left");
K.SetTransition("obs_front", "obstacle_right", "obs_right");
K.SetTransition("obs_front", "obstacle_front", "obs_front");
K.SetTransition("obs_front", "move_backward", "stuck");
K.SetTransition("obs_left", "rotate_clockwise", "clear");
K.SetTransition("obs_left", "move_forward", "clear");
K.SetTransition("obs_left", "random_walk", "obs_left");
K.SetTransition("obs_left", "crowd_detected", "crowded");
K.SetTransition("obs_left", "obstacle_front", "obs_front");
K.SetTransition("obs_left", "obstacle_right", "obs_right");
K.SetTransition("obs_left", "obstacle_left", "obs_left");
K.SetTransition("obs_left", "path_clear", "clear");
K.SetTransition("obs_left", "move_backward", "stuck");
K.SetTransition("obs_right", "rotate_counterclockwise", "clear");
K.SetTransition("obs_right", "move_forward", "clear");
K.SetTransition("obs_right", "random_walk", "obs_right");
K.SetTransition("obs_right", "crowd_detected", "crowded");
K.SetTransition("obs_right", "obstacle_front", "obs_front");
K.SetTransition("obs_right", "obstacle_left", "obs_left");
K.SetTransition("obs_right", "obstacle_right", "obs_right");
K.SetTransition("obs_right", "path_clear", "clear");
K.SetTransition("obs_right", "move_backward", "stuck");
K.SetTransition("crowded", "slow_down", "crowded");
K.SetTransition("crowded", "random_walk", "crowded");
K.SetTransition("crowded", "full_rotate", "crowded");
K.SetTransition("crowded", "crowd_detected", "crowded");
K.SetTransition("crowded", "obstacle_front", "crowded");
K.SetTransition("crowded", "obstacle_left", "crowded");
K.SetTransition("crowded", "obstacle_right", "crowded");
K.SetTransition("crowded", "path_clear", "clear");
K.SetTransition("stuck", "full_rotate", "clear");
K.SetTransition("stuck", "random_walk", "stuck");
K.SetTransition("stuck", "crowd_detected", "crowded");
K.SetTransition("stuck", "path_clear", "clear");


    return K;
}

int main() {

    try {
        // 1) Build plant and spec
        Generator G = MakeFreeRobotModel();
        Generator K = MakeSpecReactiveMotion();

        // 2) Build controllable alphabet CAlph
        EventSet CAlph;
        for (const auto& name : kControllableEvents) {
            Idx e = G.EventIndex(name);  // same indices in G and K (same names)
            CAlph.Insert(e);
        }

        // 3) Synthesize supervisor Sup (supremal controllable, nonblocking)
        Generator Sup;
        SupConNB(G, CAlph, K, Sup);          // uses controllable set CAlph
        Sup.Name("Sup_reactive_motion");

        // 4) Write models to files
        G.Write("G_robot_free_gen8.gen");
        K.Write("K_reactive_motion_gen8.gen");
        Sup.Write("Sup_reactive_motion_gen8.gen");
        Sup.DotWrite("Sup_reactive_motion_gen8.dot");

        std::cout << "Supervisor synthesis completed.\n";
    }
    catch (const Exception& e) {
        std::cerr << "FAUDES Exception: " << e.What() << std::endl;
        return 1;
    }

    return 0;
}
