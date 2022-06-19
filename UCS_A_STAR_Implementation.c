#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>

#define True 1
#define False 0
#define N 10
#define P 0.3


// Maze Matrix
struct State **GRID = NULL;


// Search Frontier
struct Node *SEARCH_FRONTIER_HEAD = NULL;
int SEARCH_FRONTIER_NODES_COUNT = 0;
int NUMBER_OF_PATHS_TO_NODES_CREATED = 0;


// Closed Set
struct Node *CLOSED_SET_HEAD = NULL;
int CLOSED_SET_NODES_COUNT = 0;


// Structs Declarations
struct State {
    int value;
    int is_free;
    double distance_from_initial_state;
};
struct Node {
    int i, j;
    struct State *state;
    struct Node *next;
    struct Node *predecessor;
};


// Methods Declarations
struct Node *get_node_with_minimum_distance_state_in_search_frontier();

int is_node_with_goal_state(struct Node *, const int *, const int *, int *);

int initial_state_is_goal_state(const int *, const int *, const int *);

struct Node *Uniform_Cost_Search_Algorithm(int *, int *, int *, int *);

struct Node *A_STAR_algorithm(int *, int *, int *, int *);

int state_is_not_in_search_frontier(struct State *);

int state_is_not_in_closed_set(struct State *);

int read_user_input(int *, int *, int *);

int max(int, int);

double heuristic_function(int, int, int *, int *);

double min(double, double);

void create_and_add_node_to_a_set(struct Node **set_head, struct State *state, int *nodes_count, int i, int j,
                                  struct Node *);

void add_children_to_search_frontier(int, int, struct Node *, int *, int *, int);

void UCS_OR_A_STAR_solution_handler(struct Node *, int, double);

void add_Node_in_closed_set(struct Node *node);

void algorithm_selection(int, int *, int *, int *);

void free_linked_list(struct Node **, int *);

void switch_case_A_STAR(int *, int *, int *);

void switch_case_UCS(int *, int *, int *);

void switch_case_BOTH(int *, int *, int *);

void free_states(struct State **grid);

void print_nodes_in_search_frontier();

void print_nodes_in_closed_set();

void reset_distances_grid();

void program_start_prints();

void create_grid();

void print_grid();


int main(void) {

    int s[2], g_1[2], g_2[2], algorithm_ID;
    program_start_prints();
    algorithm_ID = read_user_input(s, g_1, g_2);
    algorithm_selection(algorithm_ID, s, g_1, g_2);
    printf("||  Program Finished  ||\n");

    free_linked_list(&SEARCH_FRONTIER_HEAD, &SEARCH_FRONTIER_NODES_COUNT);
    free_linked_list(&CLOSED_SET_HEAD, &CLOSED_SET_NODES_COUNT);
    free_states(GRID);
    return 0;
}





///////////////////////////////////////////////////////////////////////////////////////

void reset_distances_grid() {
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            GRID[i][j].distance_from_initial_state = 0;
}

void program_start_prints() {
    printf("||    Maze Searching Program Started   ||\n\n");
    printf("Preset Settings :\nMatrix Size N = %d\nProbability for blocked state in GRID P = %lf\n", N, P);
    printf("\nGenerating Grid....\n\n");
    create_grid();
    print_grid();
}

void create_grid() {
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    srand(current_time.tv_usec);

    GRID = (struct State **) malloc(N * sizeof(struct State *));
    for (int i = 0; i < N; i++) {
        GRID[i] = (struct State *) malloc(N * sizeof(struct State));
        for (int j = 0; j < N; j++) {
            GRID[i][j].value = rand() % 4 + 1;
            GRID[i][j].distance_from_initial_state = 0;
            int probability = rand() % 100;
            GRID[i][j].is_free = (probability < P * 100) ? False : True;
        }
    }
}

void algorithm_selection(int method_ID, int *initial_state_S, int *final_state_G1, int *final_state_G2) {
    printf("METHOD ID %d\n", method_ID);
    switch (method_ID) {
        case 1:
            switch_case_UCS(initial_state_S, final_state_G1, final_state_G2);
            break;
        case 2:
            switch_case_A_STAR(initial_state_S, final_state_G1, final_state_G2);
            break;
        case 3:
            switch_case_BOTH(initial_state_S, final_state_G1, final_state_G2);
            break;
        default:
            printf("Wrong method ID number inputted!\n");
            printf("Error!");
            exit(-1);
    }
}


void switch_case_UCS(int *initial_state_S, int *final_state_G1, int *final_state_G2) {
    int final_state_found_ID;
    double process_time;
    struct Node *final_state_node;
    struct timeval start, end;

    printf("\nUniform Cost Search Algorithm Calculation Begins\n");
    printf("                     *\n""                     *\n""                     *\n");
    gettimeofday(&start, NULL);
    final_state_node = Uniform_Cost_Search_Algorithm(initial_state_S, final_state_G1, final_state_G2,
                                                     &final_state_found_ID);
    gettimeofday(&end, NULL);
    process_time = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) * 1.0E-9;

    if (final_state_node != NULL) UCS_OR_A_STAR_solution_handler(final_state_node, final_state_found_ID, process_time);
}

void switch_case_A_STAR(int *initial_state_S, int *final_state_G1, int *final_state_G2) {
    int final_state_found_ID;
    double process_time;
    struct Node *final_state_node;
    struct timeval start, end;

    printf("\nA_STAR Cost Search Algorithm Calculation Begins\n");
    printf("                     *\n""                     *\n""                     *\n");
    gettimeofday(&start, NULL);
    final_state_node = A_STAR_algorithm(initial_state_S, final_state_G1, final_state_G2, &final_state_found_ID);
    gettimeofday(&end, NULL);
    process_time = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) * 1.0E-9;

    if (final_state_node != NULL) UCS_OR_A_STAR_solution_handler(final_state_node, final_state_found_ID, process_time);

}

void switch_case_BOTH(int *initial_state_S, int *final_state_G1, int *final_state_G2) {
    switch_case_UCS(initial_state_S, final_state_G1, final_state_G2);
    NUMBER_OF_PATHS_TO_NODES_CREATED = 0;
    free_linked_list(&SEARCH_FRONTIER_HEAD, &SEARCH_FRONTIER_NODES_COUNT);
    free_linked_list(&CLOSED_SET_HEAD, &CLOSED_SET_NODES_COUNT);
    reset_distances_grid();
    printf("\n\n------------------------------------------------------------------------------\n"
           "------------------------------------------------------------------------------\n"
           "------------------------------------------------------------------------------\n"
           "------------------------------------------------------------------------------\n"
           "------------------------------------------------------------------------------\n"
           "------------------------------------------------------------------------------\n\n\n");
    switch_case_A_STAR(initial_state_S, final_state_G1, final_state_G2);
}


struct Node *Uniform_Cost_Search_Algorithm(int *initial_state_S, int *final_state_G1, int *final_state_G2,
                                           int *final_state_found_ID) {
    if (!initial_state_is_goal_state(initial_state_S, final_state_G1, final_state_G2)) {
        struct State *start_state = &GRID[initial_state_S[0]][initial_state_S[1]];
        create_and_add_node_to_a_set(&CLOSED_SET_HEAD, start_state, &CLOSED_SET_NODES_COUNT,
                                     initial_state_S[0], initial_state_S[1], NULL);

        add_children_to_search_frontier(initial_state_S[0], initial_state_S[1], CLOSED_SET_HEAD,
                                        final_state_G1, final_state_G2, 0);

        while (SEARCH_FRONTIER_NODES_COUNT > 0) {
            struct Node *node_with_minimum_distance = get_node_with_minimum_distance_state_in_search_frontier();

            if (is_node_with_goal_state(node_with_minimum_distance, final_state_G1, final_state_G2,
                                        final_state_found_ID))
                return node_with_minimum_distance;

            add_Node_in_closed_set(node_with_minimum_distance);
            add_children_to_search_frontier(node_with_minimum_distance->i, node_with_minimum_distance->j,
                                            node_with_minimum_distance, final_state_G1, final_state_G2, 0);
        }
        printf("Uniform Cost Search Algorithm Finished\nSearch frontier list got empty before a solution Path found\n");
        printf("No free path from initial state to some of final states\nReduce probability P! OR "
               "check if both of the final states are blocked states\n");
        return NULL;
    } else {
        printf("Initial State is one of the goal States\n");
        printf("No algorithm Execution");
        return NULL;
    }
}

struct Node *
A_STAR_algorithm(int *initial_state_S, int *final_state_G1, int *final_state_G2, int *final_state_found_ID) {
    if (!initial_state_is_goal_state(initial_state_S, final_state_G1, final_state_G2)) {
        struct State *start_state = &GRID[initial_state_S[0]][initial_state_S[1]];
        create_and_add_node_to_a_set(&CLOSED_SET_HEAD, start_state, &CLOSED_SET_NODES_COUNT,
                                     initial_state_S[0], initial_state_S[1], NULL);

        add_children_to_search_frontier(initial_state_S[0], initial_state_S[1], CLOSED_SET_HEAD,
                                        final_state_G1, final_state_G2, 1);


        while (SEARCH_FRONTIER_NODES_COUNT > 0) {
            struct Node *node_with_minimum_distance = get_node_with_minimum_distance_state_in_search_frontier();

            if (is_node_with_goal_state(node_with_minimum_distance, final_state_G1, final_state_G2,
                                        final_state_found_ID))
                return node_with_minimum_distance;

            add_Node_in_closed_set(node_with_minimum_distance);
            add_children_to_search_frontier(node_with_minimum_distance->i, node_with_minimum_distance->j,
                                            node_with_minimum_distance, final_state_G1, final_state_G2, 1);
        }
        printf("A_START Algorithm Finished\nSearch frontier list got empty before a solution Path found\n");
        printf("No free path from initial state to some of final states\nReduce probability P! OR "
               "check if both of the final states are blocked states\n");
        return NULL;
    } else {
        printf("Initial State is one of the goal States\n");
        printf("No algorithm Execution");
        return NULL;
    }

}


// Methods for algorithms implementation
void UCS_OR_A_STAR_solution_handler(struct Node *final_state_node, int final_state_ID, double process_time) {
    printf("\n\nFinal State G %d found as the shortest path solution\n", final_state_ID);
    printf("Printing solution path going throw the root ...\n\n\n");

    struct Node *current_node = final_state_node;
    double cost_of_final_state_path = current_node->state->distance_from_initial_state;

    while (current_node != NULL) {
        printf("----------------\n");
        printf("State Coordinates -> (%d,%d)\n", current_node->i, current_node->j);
        printf("State Value -> %d\n", current_node->state->value);
        printf("State Distance from initial state -> %lf\n", current_node->state->distance_from_initial_state);
        printf("----------------\n\n");

        current_node = current_node->predecessor;
    }
    printf("Cost form initial state to final state G_%d is %lf\n", final_state_ID, cost_of_final_state_path);
    printf("Number of paths to states created in order to find shortest path %d\n", NUMBER_OF_PATHS_TO_NODES_CREATED);
    printf("Number of states visited (Closed Set length) -> %d\n", CLOSED_SET_NODES_COUNT);
    printf("Process Time of the algorithm -> %.15lf seconds\n\n", process_time);
}

int state_is_not_in_search_frontier(struct State *state_to_check) {
    struct Node *node = SEARCH_FRONTIER_HEAD;
    while (node != NULL) {
        if (node->state == state_to_check)
            return False;

        node = node->next;
    }
    return True;
}

int is_node_with_goal_state(struct Node *node_to_check, const int *final_state_G1, const int *final_state_G2,
                            int *final_state_ID) {
    int state_grid_position[2] = {node_to_check->i, node_to_check->j};
    if (state_grid_position[0] == final_state_G1[0] && state_grid_position[1] == final_state_G1[1]) {
        *(final_state_ID) = 1;
        return True;
    } else if (state_grid_position[0] == final_state_G2[0] && state_grid_position[1] == final_state_G2[1]) {
        *(final_state_ID) = 2;
        return True;
    } else
        return False;
}


struct Node *get_node_with_minimum_distance_state_in_search_frontier() {
    struct Node *current_node = SEARCH_FRONTIER_HEAD;

    // Case list has one node
    if (SEARCH_FRONTIER_NODES_COUNT == 1) {
        SEARCH_FRONTIER_HEAD = NULL;
        SEARCH_FRONTIER_NODES_COUNT--;
        return current_node;
    }

    struct Node *node_with_minimum_distance_state = SEARCH_FRONTIER_HEAD;
    struct Node *previous_node_from_deleted_node = NULL;
    struct Node *previous_node = SEARCH_FRONTIER_HEAD;
    current_node = current_node->next;

    while (current_node != NULL) {
        if (current_node->state->distance_from_initial_state <
            node_with_minimum_distance_state->state->distance_from_initial_state) {
            node_with_minimum_distance_state = current_node;
            previous_node_from_deleted_node = previous_node;
        }

        previous_node = previous_node->next;
        current_node = current_node->next;
    }

    // Case when first node is one to be deleted
    if (previous_node_from_deleted_node == NULL)
        SEARCH_FRONTIER_HEAD = SEARCH_FRONTIER_HEAD->next;
    else
        previous_node_from_deleted_node->next = node_with_minimum_distance_state->next;


    node_with_minimum_distance_state->next = NULL;
    SEARCH_FRONTIER_NODES_COUNT--;
    return node_with_minimum_distance_state;
}

void add_children_to_search_frontier(int i, int j, struct Node *predecessor, int *final_state_G1, int *final_state_G2,
                                     int algorithm_ID) {

    struct State *current_state = &GRID[i][j];
    if (j + 1 < N) {
        struct State *neighbor_state = &GRID[i][j + 1];

        if (neighbor_state->is_free && state_is_not_in_closed_set(neighbor_state)
            && state_is_not_in_search_frontier(neighbor_state)) {
            neighbor_state->distance_from_initial_state =
                    abs(current_state->value - neighbor_state->value)
                    + 1 + current_state->distance_from_initial_state
                    + (algorithm_ID ? heuristic_function(i, j, final_state_G1, final_state_G2) : 0);

            create_and_add_node_to_a_set(&SEARCH_FRONTIER_HEAD, neighbor_state,
                                         &SEARCH_FRONTIER_NODES_COUNT, i, j + 1, predecessor);
            NUMBER_OF_PATHS_TO_NODES_CREATED++;
        }
    }


    if (j - 1 >= 0) {
        struct State *neighbor_state = &GRID[i][j - 1];

        if (neighbor_state->is_free && state_is_not_in_closed_set(neighbor_state)
            && state_is_not_in_search_frontier(neighbor_state)) {
            neighbor_state->distance_from_initial_state =
                    abs(current_state->value - neighbor_state->value)
                    + 1 + current_state->distance_from_initial_state
                    + (algorithm_ID ? heuristic_function(i, j, final_state_G1, final_state_G2) : 0);


            create_and_add_node_to_a_set(&SEARCH_FRONTIER_HEAD, neighbor_state,
                                         &SEARCH_FRONTIER_NODES_COUNT, i, j - 1, predecessor);
            NUMBER_OF_PATHS_TO_NODES_CREATED++;
        }
    }


    if (i + 1 < N) {
        struct State *neighbor_state = &GRID[i + 1][j];

        if (neighbor_state->is_free && state_is_not_in_closed_set(neighbor_state)
            && state_is_not_in_search_frontier(neighbor_state)) {
            neighbor_state->distance_from_initial_state =
                    abs(current_state->value - neighbor_state->value)
                    + 1 + current_state->distance_from_initial_state
                    + (algorithm_ID ? heuristic_function(i, j, final_state_G1, final_state_G2) : 0);


            create_and_add_node_to_a_set(&SEARCH_FRONTIER_HEAD, neighbor_state,
                                         &SEARCH_FRONTIER_NODES_COUNT, i + 1, j, predecessor);
            NUMBER_OF_PATHS_TO_NODES_CREATED++;
        }
        if (j + 1 < N) {
            struct State *neighbor_state = &GRID[i + 1][j + 1];

            if (neighbor_state->is_free && state_is_not_in_closed_set(neighbor_state)
                && state_is_not_in_search_frontier(neighbor_state)) {
                neighbor_state->distance_from_initial_state =
                        abs(current_state->value - neighbor_state->value)
                        + 0.5 + current_state->distance_from_initial_state
                        + (algorithm_ID ? heuristic_function(i, j, final_state_G1, final_state_G2) : 0);


                create_and_add_node_to_a_set(&SEARCH_FRONTIER_HEAD, neighbor_state,
                                             &SEARCH_FRONTIER_NODES_COUNT, i + 1, j + 1, predecessor);
                NUMBER_OF_PATHS_TO_NODES_CREATED++;
            }
        }
        if (j - 1 >= 0) {
            struct State *neighbor_state = &GRID[i + 1][j - 1];

            if (neighbor_state->is_free && state_is_not_in_closed_set(neighbor_state)
                && state_is_not_in_search_frontier(neighbor_state)) {
                neighbor_state->distance_from_initial_state =
                        abs(current_state->value - neighbor_state->value)
                        + 0.5 + current_state->distance_from_initial_state
                        + (algorithm_ID ? heuristic_function(i, j, final_state_G1, final_state_G2) : 0);

                create_and_add_node_to_a_set(&SEARCH_FRONTIER_HEAD, neighbor_state,
                                             &SEARCH_FRONTIER_NODES_COUNT, i + 1, j - 1, predecessor);
                NUMBER_OF_PATHS_TO_NODES_CREATED++;
            }
        }
    }


    if (i - 1 >= 0) {
        struct State *neighbor_state = &GRID[i - 1][j];

        if (neighbor_state->is_free && state_is_not_in_closed_set(neighbor_state)
            && state_is_not_in_search_frontier(neighbor_state)) {
            neighbor_state->distance_from_initial_state =
                    abs(current_state->value - neighbor_state->value)
                    + 1 + current_state->distance_from_initial_state
                    + (algorithm_ID ? heuristic_function(i, j, final_state_G1, final_state_G2) : 0);

            create_and_add_node_to_a_set(&SEARCH_FRONTIER_HEAD, neighbor_state,
                                         &SEARCH_FRONTIER_NODES_COUNT, i - 1, j, predecessor);
            NUMBER_OF_PATHS_TO_NODES_CREATED++;
        }
        if (j + 1 < N) {
            struct State *neighbor_state = &GRID[i - 1][j + 1];

            if (neighbor_state->is_free && state_is_not_in_closed_set(neighbor_state)
                && state_is_not_in_search_frontier(neighbor_state)) {
                neighbor_state->distance_from_initial_state =
                        abs(current_state->value - neighbor_state->value)
                        + 0.5 + current_state->distance_from_initial_state
                        + (algorithm_ID ? heuristic_function(i, j, final_state_G1, final_state_G2) : 0);


                create_and_add_node_to_a_set(&SEARCH_FRONTIER_HEAD, neighbor_state,
                                             &SEARCH_FRONTIER_NODES_COUNT, i - 1, j + 1, predecessor);
                NUMBER_OF_PATHS_TO_NODES_CREATED++;
            }
        }
        if (j - 1 >= 0) {
            struct State *neighbor_state = &GRID[i - 1][j - 1];

            if (neighbor_state->is_free && state_is_not_in_closed_set(neighbor_state)
                && state_is_not_in_search_frontier(neighbor_state)) {
                neighbor_state->distance_from_initial_state =
                        abs(current_state->value - neighbor_state->value)
                        + 0.5 + current_state->distance_from_initial_state
                        + (algorithm_ID ? heuristic_function(i, j, final_state_G1, final_state_G2) : 0);


                create_and_add_node_to_a_set(&SEARCH_FRONTIER_HEAD, neighbor_state,
                                             &SEARCH_FRONTIER_NODES_COUNT, i - 1, j - 1, predecessor);
                NUMBER_OF_PATHS_TO_NODES_CREATED++;
            }
        }
    }
}


void create_and_add_node_to_a_set(struct Node **set_head, struct State *state, int *nodes_count, int i, int j,
                                  struct Node *predecessor) {
    struct Node *new_node = (struct Node *) malloc(1 * sizeof(struct Node));
    new_node->i = i;
    new_node->j = j;
    new_node->state = state;
    new_node->predecessor = predecessor;
    new_node->next = (*nodes_count < 1) ? NULL : *set_head;
    *set_head = new_node;
    (*nodes_count)++;
}

int state_is_not_in_closed_set(struct State *state_to_check) {
    struct Node *node = CLOSED_SET_HEAD;
    while (node != NULL) {
        if (node->state == state_to_check)
            return False;
        node = node->next;
    }
    return True;
}

void add_Node_in_closed_set(struct Node *node) {
    if (CLOSED_SET_NODES_COUNT++ < 1)
        CLOSED_SET_HEAD = node;
    else {
        node->next = CLOSED_SET_HEAD;
        CLOSED_SET_HEAD = node;
    }
}

int initial_state_is_goal_state(const int *current_state, const int *final_state_G1, const int *final_state_G2) {
    return ((current_state[0] == final_state_G1[0] && current_state[1] == final_state_G1[1]) ||
            (current_state[0] == final_state_G2[0] && current_state[1] == final_state_G2[1]));
}

void free_linked_list(struct Node **head_address, int *nodes_count) {
    struct Node *temp, *head, **keep_old_head;
    head = *head_address;
    keep_old_head = head_address;
    while (head != NULL) {
        temp = head;
        head = head->next;
        free(temp);
    }
    *(nodes_count) = 0;
    *keep_old_head = NULL;
}

void free_states(struct State **grid) {
    for (int i = 0; i < N; i++)
        free(grid[i]);
    free(grid);

    GRID = NULL;
}

int read_user_input(int *initial_state_S, int *final_state_G1, int *final_state_G2) {
    printf("Input the coordinates of initial_state , final_state_g1 , final_state_g2\n");
    printf("---------------------------\nNote that coordinates that lead to a blocked state "
           "(according to the grid created above) cannot "
           "be given as input!\nThat will lead to program failure\n---------------------------");
    printf("\nInput the coordinates of the initial state S:\n");
    printf("[Format -> 'x,y']\n");
    printf("Answer:");
    scanf("%d,%d", &initial_state_S[0], &initial_state_S[1]);
    printf("\nStarting Position S selected as the (%d,%d)\n-----\n", initial_state_S[0], initial_state_S[1]);
    printf("Input G1 final state coordinates:\n");
    printf("[Format -> 'x,y']\n");
    printf("Answer:");
    scanf("%d,%d", &final_state_G1[0], &final_state_G1[1]);
    printf("\nFinal State G1 selected as the (%d,%d)\n-----\n", final_state_G1[0], final_state_G1[1]);
    printf("Input the G2 Final State coordinates:\n");
    printf("[Format -> 'x,y']\n");
    printf("Answer:");
    scanf("%d,%d", &final_state_G2[0], &final_state_G2[1]);
    printf("\nFinal State G1 selected as the (%d,%d)\n-----\n", final_state_G2[0], final_state_G2[1]);
    printf("\nSelect what method is to be used:           [Input corresponding number]\n");
    printf("1) Uniform Cost Search [UCS]\n");
    printf("2) A*:\n");
    printf("3) Both:\n");
    printf("Answer:");
    int method_ID;
    scanf("%d", &method_ID);
    return method_ID;
}

double heuristic_function(int i, int j, int *final_state_G1, int *final_state_G2) {
    double dx_1, dy_1, dx_2, dy_2;

    dx_1 = abs(final_state_G1[0] - i);
    dx_2 = abs(final_state_G2[0] - i);

    dy_1 = abs(final_state_G1[1] - j);
    dy_2 = abs(final_state_G2[1] - j);

    return min(2 * (dx_1 + dy_1) + (sqrt(2) - 2) * min(dx_1, dy_1),
               2 * (dx_2 + dy_2) + (sqrt(2) - 2) * min(dx_2, dy_2));

}

int max(int num_1, int num_2) {
    return num_1 > num_2 ? num_1 : num_2;
}

double min(double num_1, double num_2) {
    return num_1 < num_2 ? num_1 : num_2;
}

// Print Methods
void print_grid() {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            char *state = GRID[i][j].is_free ? "FREE   " : "BLOCKED";
            printf("||(%d,%d) Value:%d   %s  ||     ", i, j, GRID[i][j].value, state);
        }
        printf("\n\n");
    }
}

void print_nodes_in_search_frontier() {
    struct Node *node = SEARCH_FRONTIER_HEAD;
    printf("Nodes in search Frontier:\n\n");
    int count = 1;
    while (node != NULL) {
        printf("Node Count : %d\n", count++);
        printf("State position in GRID -> (%d,%d)\n", node->i, node->j);
        printf("State Value : %d\n", node->state->value);
        printf("State Distance from initial State : %lf\n", node->state->distance_from_initial_state);
        node = node->next;
        printf("-------------------------------------------------------\n");
    }
}

void print_nodes_in_closed_set() {
    struct Node *node = CLOSED_SET_HEAD;
    printf("Closed Set Nodes :\n");
    int count = 1;
    while (node != NULL) {
        printf("Node %d\n", count++);
        printf("State Position in GRID ->(%d,%d)\n", node->i, node->j);
        printf("State value: %d\n", node->state->value);
        printf("State distance: %lf\n", node->state->distance_from_initial_state);
        printf("--------------------\n");
        node = node->next;
    }
}

