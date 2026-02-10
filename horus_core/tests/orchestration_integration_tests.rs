//! Orchestration Integration Tests
//!
//! Comprehensive integration tests for the Mission Planner and State Machines modules,
//! demonstrating real robotics workflow patterns including:
//! - Pick-and-place operations
//! - Navigation with replanning
//! - State machine + mission planner integration
//! - Failure recovery and retry patterns
//! - Parallel task execution
//! - Conditional execution based on sensor data

use horus_core::mission_planner::{
    ExecutionContext, ExecutionStatus, GoalFailurePolicy, GoalSpec, MissionEvent, MissionMode,
    MissionPlanner, MissionPlannerBuilder, MissionSpec, Priority, RetryPolicy, TaskCondition,
    TaskExecutor, TaskSpec,
};
use horus_core::serde_json::{self, json};
use horus_core::state_machines::{
    Event, EventPriority, SharedStateMachine, State, StateMachine, StateMachineBuilder, Transition,
};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Helper to extract parameters from a TaskSpec's executor
fn get_executor_params(task: &TaskSpec) -> Option<&HashMap<String, serde_json::Value>> {
    match &task.executor {
        TaskExecutor::Custom { parameters, .. } => Some(parameters),
        TaskExecutor::Action { parameters, .. } => Some(parameters),
        _ => None,
    }
}

/// Helper to convert json! value to HashMap for executor parameters
fn json_to_hashmap(value: serde_json::Value) -> HashMap<String, serde_json::Value> {
    if let serde_json::Value::Object(map) = value {
        map.into_iter().collect()
    } else {
        HashMap::new()
    }
}

// =============================================================================
// Test 1: Pick-and-Place Workflow
// =============================================================================

/// Simulates a complete pick-and-place robotics workflow with:
/// - Navigation to pick location
/// - Object detection
/// - Gripper close
/// - Navigation to place location
/// - Gripper open
#[test]
fn test_pick_and_place_workflow() {
    // Execution log to verify task order
    let execution_log = Arc::new(Mutex::new(Vec::<String>::new()));

    // Simulated robot state
    let robot_position = Arc::new(Mutex::new([0.0f64, 0.0, 0.0])); // x, y, z
    let gripper_closed = Arc::new(AtomicBool::new(false));
    let object_held = Arc::new(AtomicBool::new(false));

    // Create mission planner with custom executors
    let mut planner = MissionPlannerBuilder::new()
        .max_concurrent_tasks(2)
        .collect_metrics(true)
        .build();

    // Register navigate executor
    let log_nav = Arc::clone(&execution_log);
    let pos_nav = Arc::clone(&robot_position);
    planner.register_executor(
        "navigate",
        Arc::new(move |task: &TaskSpec, _ctx: &ExecutionContext| {
            let params = get_executor_params(task);
            let target = params
                .and_then(|p| p.get("target"))
                .and_then(|v| v.as_array());
            if let Some(coords) = target {
                let x = coords.get(0).and_then(|v| v.as_f64()).unwrap_or(0.0);
                let y = coords.get(1).and_then(|v| v.as_f64()).unwrap_or(0.0);
                let z = coords.get(2).and_then(|v| v.as_f64()).unwrap_or(0.0);

                // Simulate navigation
                let mut pos = pos_nav.lock().unwrap();
                pos[0] = x;
                pos[1] = y;
                pos[2] = z;

                let mut log = log_nav.lock().unwrap();
                log.push(format!("navigate_to({:.1},{:.1},{:.1})", x, y, z));

                Ok(Some(json!({
                    "success": true,
                    "position": [x, y, z]
                })))
            } else {
                Err("Missing target coordinates".to_string())
            }
        }),
    );

    // Register detect_object executor
    let log_detect = Arc::clone(&execution_log);
    planner.register_executor(
        "detect_object",
        Arc::new(move |_task: &TaskSpec, _ctx: &ExecutionContext| {
            let mut log = log_detect.lock().unwrap();
            log.push("detect_object".to_string());

            Ok(Some(json!({
                "detected": true,
                "object_id": "box_001",
                "confidence": 0.95
            })))
        }),
    );

    // Register gripper executor
    let log_gripper = Arc::clone(&execution_log);
    let gripper_state = Arc::clone(&gripper_closed);
    let object_state = Arc::clone(&object_held);
    planner.register_executor(
        "gripper",
        Arc::new(move |task: &TaskSpec, _ctx: &ExecutionContext| {
            let params = get_executor_params(task);
            let action = params
                .and_then(|p| p.get("action"))
                .and_then(|v| v.as_str())
                .unwrap_or("close");

            let mut log = log_gripper.lock().unwrap();
            log.push(format!("gripper_{}", action));

            match action {
                "close" => {
                    gripper_state.store(true, Ordering::SeqCst);
                    object_state.store(true, Ordering::SeqCst);
                    Ok(Some(json!({"closed": true, "force": 10.0})))
                }
                "open" => {
                    gripper_state.store(false, Ordering::SeqCst);
                    object_state.store(false, Ordering::SeqCst);
                    Ok(Some(json!({"closed": false})))
                }
                _ => Err(format!("Unknown gripper action: {}", action)),
            }
        }),
    );

    // Build the pick-and-place mission
    // Task definitions
    let nav_to_pick = TaskSpec::new(
        "nav_pick",
        "Navigate to pick location",
        TaskExecutor::Custom {
            handler: "navigate".to_string(),
            parameters: json_to_hashmap(json!({"target": [1.0, 2.0, 0.5]})),
        },
    );

    let detect = TaskSpec::new(
        "detect",
        "Detect object",
        TaskExecutor::Custom {
            handler: "detect_object".to_string(),
            parameters: HashMap::new(),
        },
    );

    let gripper_close = TaskSpec::new(
        "gripper_close",
        "Close gripper to grab object",
        TaskExecutor::Custom {
            handler: "gripper".to_string(),
            parameters: json_to_hashmap(json!({"action": "close"})),
        },
    );

    let nav_to_place = TaskSpec::new(
        "nav_place",
        "Navigate to place location",
        TaskExecutor::Custom {
            handler: "navigate".to_string(),
            parameters: json_to_hashmap(json!({"target": [3.0, 4.0, 0.5]})),
        },
    );

    let gripper_open = TaskSpec::new(
        "gripper_open",
        "Open gripper to release object",
        TaskExecutor::Custom {
            handler: "gripper".to_string(),
            parameters: json_to_hashmap(json!({"action": "open"})),
        },
    );

    // Build goal with task dependencies
    let nav_pick_id = nav_to_pick.id.clone();
    let detect_id = detect.id.clone();
    let gripper_close_id = gripper_close.id.clone();
    let nav_place_id = nav_to_place.id.clone();

    let pick_goal = GoalSpec::new("pick", "Pick up object")
        .add_task(nav_to_pick)
        .add_task_after(detect, &nav_pick_id)
        .add_task_after(gripper_close, &detect_id);

    let place_goal = GoalSpec::new("place", "Place object")
        .add_task(nav_to_place)
        .add_task_after(gripper_open, &nav_place_id);

    let pick_goal_id = pick_goal.id.clone();

    let mission = MissionSpec::with_id("pick_and_place", "Pick and place mission")
        .with_description("Complete pick and place workflow")
        .add_goal(pick_goal)
        .add_goal_after(place_goal, &pick_goal_id);

    // Submit and execute
    let mission_id = planner.submit(mission).unwrap();
    planner.start(&mission_id).unwrap();

    // Tick until complete
    for _ in 0..50 {
        planner.tick().unwrap();
        let status = planner.get_mission_status(&mission_id).unwrap();
        if status.status == ExecutionStatus::Completed {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }

    // Verify completion
    let status = planner.get_mission_status(&mission_id).unwrap();
    assert_eq!(status.status, ExecutionStatus::Completed);

    // Verify execution order
    let log = execution_log.lock().unwrap();
    assert_eq!(log.len(), 5);
    assert_eq!(log[0], "navigate_to(1.0,2.0,0.5)");
    assert_eq!(log[1], "detect_object");
    assert_eq!(log[2], "gripper_close");
    assert_eq!(log[3], "navigate_to(3.0,4.0,0.5)");
    assert_eq!(log[4], "gripper_open");

    // Verify final robot state
    let final_pos = robot_position.lock().unwrap();
    assert_eq!(final_pos[0], 3.0);
    assert_eq!(final_pos[1], 4.0);
    assert!(!gripper_closed.load(Ordering::SeqCst));
    assert!(!object_held.load(Ordering::SeqCst));
}

// =============================================================================
// Test 2: Navigation with Replanning
// =============================================================================

/// Tests navigation with obstacle detection and path replanning.
/// Simulates:
/// - Initial navigation attempt
/// - Obstacle detected mid-path
/// - Retry with alternate route
#[test]
fn test_navigation_with_replanning() {
    let attempt_count = Arc::new(AtomicUsize::new(0));
    let path_blocked = Arc::new(AtomicBool::new(true)); // Initially blocked

    let mut planner = MissionPlanner::new();

    // Register navigate executor with simulated obstacle
    let attempts = Arc::clone(&attempt_count);
    let blocked = Arc::clone(&path_blocked);
    planner.register_executor(
        "navigate_with_obstacle",
        Arc::new(move |_task: &TaskSpec, _ctx: &ExecutionContext| {
            let current_attempt = attempts.fetch_add(1, Ordering::SeqCst);

            if current_attempt == 0 && blocked.load(Ordering::SeqCst) {
                // First attempt fails due to obstacle
                Err("Path blocked by obstacle".to_string())
            } else {
                // Second attempt succeeds (assuming path was replanned)
                blocked.store(false, Ordering::SeqCst);
                Ok(Some(json!({
                    "success": true,
                    "path": "alternate_route"
                })))
            }
        }),
    );

    // Create task with retry policy
    let nav_task = TaskSpec::new(
        "navigate",
        "Navigate to goal",
        TaskExecutor::Custom {
            handler: "navigate_with_obstacle".to_string(),
            parameters: HashMap::new(),
        },
    )
    .with_retry(RetryPolicy {
        max_attempts: 3,
        delay: Duration::from_millis(10),
        exponential_backoff: false,
        max_delay: Duration::from_millis(100),
        retry_on: vec!["Path blocked".to_string()],
    });

    let goal = GoalSpec::new("nav_goal", "Navigation goal").add_task(nav_task);

    let mission = MissionSpec::with_id("nav_mission", "Navigation with replanning").add_goal(goal);

    let id = planner.submit(mission).unwrap();
    planner.start(&id).unwrap();

    // Run with enough ticks for retry
    for _ in 0..30 {
        planner.tick().unwrap();
        let status = planner.get_mission_status(&id).unwrap();
        if status.status == ExecutionStatus::Completed || status.status == ExecutionStatus::Failed {
            break;
        }
        std::thread::sleep(Duration::from_millis(15));
    }

    let status = planner.get_mission_status(&id).unwrap();
    assert_eq!(status.status, ExecutionStatus::Completed);
    assert!(attempt_count.load(Ordering::SeqCst) >= 2); // At least 2 attempts
}

// =============================================================================
// Test 3: State Machine + Mission Planner Integration
// =============================================================================

/// Tests integration between State Machines and Mission Planner.
/// The state machine controls robot mode, and missions are only
/// executed when in the appropriate mode.
#[test]
fn test_state_machine_mission_planner_integration() {
    // Robot mode state machine
    struct RobotContext {
        battery_level: f32,
        is_docked: bool,
        mission_active: bool,
    }

    let fsm = StateMachineBuilder::<RobotContext>::new("robot_mode")
        .initial_state("idle")
        .on_entry(|ctx| ctx.mission_active = false)
        .done()
        .state("executing")
        .on_entry(|ctx| ctx.mission_active = true)
        .done()
        .state("docking")
        .on_entry(|ctx| {
            ctx.is_docked = true;
            ctx.mission_active = false;
        })
        .done()
        .state("error")
        .on_entry(|ctx| ctx.mission_active = false)
        .done()
        .transition("idle", "executing")
        .on_event("start_mission")
        .with_guard(|ctx| ctx.battery_level > 0.2)
        .done()
        .transition("executing", "idle")
        .on_event("mission_complete")
        .done()
        .transition("executing", "docking")
        .on_event("low_battery")
        .with_guard(|ctx| ctx.battery_level < 0.15)
        .done()
        .transition("executing", "error")
        .on_event("error")
        .done()
        .transition("docking", "idle")
        .on_event("docked")
        .done()
        .transition("error", "idle")
        .on_event("reset")
        .done()
        .build()
        .unwrap();

    let mut ctx = RobotContext {
        battery_level: 0.8,
        is_docked: false,
        mission_active: false,
    };

    let mut fsm = fsm;
    fsm.start(&mut ctx).unwrap();

    // Verify initial state
    assert!(!ctx.mission_active);

    // Transition to executing mode
    fsm.process_event(&Event::new("start_mission"), &mut ctx)
        .unwrap();
    assert!(ctx.mission_active);

    // Now set up mission planner
    let mut planner = MissionPlanner::new();

    let task = TaskSpec::noop("work", "Do work");
    let goal = GoalSpec::new("work_goal", "Work goal").add_task(task);
    let mission = MissionSpec::with_id("work_mission", "Work mission").add_goal(goal);

    let mission_id = planner.submit(mission).unwrap();
    planner.start(&mission_id).unwrap();

    // Execute only if state machine is in executing mode
    if ctx.mission_active {
        for _ in 0..10 {
            planner.tick().unwrap();
        }
    }

    let status = planner.get_mission_status(&mission_id).unwrap();
    assert_eq!(status.status, ExecutionStatus::Completed);

    // Complete mission and transition back
    fsm.process_event(&Event::new("mission_complete"), &mut ctx)
        .unwrap();
    assert!(!ctx.mission_active);
}

// =============================================================================
// Test 4: Failure Recovery and Retry
// =============================================================================

/// Tests failure recovery with exponential backoff retry.
#[test]
fn test_failure_recovery_and_retry() {
    let attempt_count = Arc::new(AtomicUsize::new(0));
    let succeed_after = 3; // Succeed on 3rd attempt

    let mut planner = MissionPlanner::new();

    let attempts = Arc::clone(&attempt_count);
    planner.register_executor(
        "flaky_task",
        Arc::new(move |_task: &TaskSpec, _ctx: &ExecutionContext| {
            let current = attempts.fetch_add(1, Ordering::SeqCst);
            if current + 1 >= succeed_after {
                Ok(Some(json!({"success": true})))
            } else {
                Err(format!("Attempt {} failed", current + 1))
            }
        }),
    );

    let task = TaskSpec::new(
        "flaky",
        "Flaky task",
        TaskExecutor::Custom {
            handler: "flaky_task".to_string(),
            parameters: HashMap::new(),
        },
    )
    .with_retry(RetryPolicy {
        max_attempts: 5,
        delay: Duration::from_millis(10),
        exponential_backoff: true,
        max_delay: Duration::from_millis(100),
        retry_on: vec![],
    });

    let goal = GoalSpec::new("retry_goal", "Retry goal").add_task(task);
    let mission = MissionSpec::with_id("retry_mission", "Retry mission").add_goal(goal);

    let id = planner.submit(mission).unwrap();
    planner.start(&id).unwrap();

    for _ in 0..50 {
        planner.tick().unwrap();
        let status = planner.get_mission_status(&id).unwrap();
        if status.status == ExecutionStatus::Completed {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }

    let status = planner.get_mission_status(&id).unwrap();
    assert_eq!(status.status, ExecutionStatus::Completed);
    assert_eq!(attempt_count.load(Ordering::SeqCst), 3);
}

// =============================================================================
// Test 5: Parallel Task Execution
// =============================================================================

/// Tests parallel execution of independent tasks.
#[test]
fn test_parallel_task_execution() {
    let task_starts = Arc::new(Mutex::new(Vec::<String>::new()));
    let task_ends = Arc::new(Mutex::new(Vec::<String>::new()));

    let mut planner = MissionPlannerBuilder::new()
        .max_concurrent_tasks(4) // Allow 4 parallel tasks
        .build();

    // Register parallel task executor
    let starts = Arc::clone(&task_starts);
    let ends = Arc::clone(&task_ends);
    planner.register_executor(
        "parallel_work",
        Arc::new(move |task: &TaskSpec, _ctx: &ExecutionContext| {
            let task_name = task.id.as_str().to_string();

            {
                let mut s = starts.lock().unwrap();
                s.push(task_name.clone());
            }

            // Simulate work
            std::thread::sleep(Duration::from_millis(50));

            {
                let mut e = ends.lock().unwrap();
                e.push(task_name);
            }

            Ok(Some(json!({"done": true})))
        }),
    );

    // Create 4 independent tasks that can run in parallel
    let task_a = TaskSpec::new(
        "a",
        "Task A",
        TaskExecutor::Custom {
            handler: "parallel_work".to_string(),
            parameters: HashMap::new(),
        },
    );
    let task_b = TaskSpec::new(
        "b",
        "Task B",
        TaskExecutor::Custom {
            handler: "parallel_work".to_string(),
            parameters: HashMap::new(),
        },
    );
    let task_c = TaskSpec::new(
        "c",
        "Task C",
        TaskExecutor::Custom {
            handler: "parallel_work".to_string(),
            parameters: HashMap::new(),
        },
    );
    let task_d = TaskSpec::new(
        "d",
        "Task D",
        TaskExecutor::Custom {
            handler: "parallel_work".to_string(),
            parameters: HashMap::new(),
        },
    );

    // All tasks are independent (no dependencies)
    let goal = GoalSpec::new("parallel", "Parallel goal")
        .add_task(task_a)
        .add_task(task_b)
        .add_task(task_c)
        .add_task(task_d);

    let mission = MissionSpec::with_id("parallel_mission", "Parallel mission").add_goal(goal);

    let id = planner.submit(mission).unwrap();
    planner.start(&id).unwrap();

    for _ in 0..30 {
        planner.tick().unwrap();
        let status = planner.get_mission_status(&id).unwrap();
        if status.status == ExecutionStatus::Completed {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }

    let status = planner.get_mission_status(&id).unwrap();
    assert_eq!(status.status, ExecutionStatus::Completed);

    // All 4 tasks should have completed
    let ends = task_ends.lock().unwrap();
    assert_eq!(ends.len(), 4);
}

// =============================================================================
// Test 6: Conditional Execution
// =============================================================================

/// Tests conditional task execution based on context parameters.
#[test]
fn test_conditional_execution() {
    let executed_tasks = Arc::new(Mutex::new(Vec::<String>::new()));

    let mut planner = MissionPlanner::new();

    let tasks = Arc::clone(&executed_tasks);
    planner.register_executor(
        "conditional_work",
        Arc::new(move |task: &TaskSpec, _ctx: &ExecutionContext| {
            let mut t = tasks.lock().unwrap();
            t.push(task.id.as_str().to_string());
            Ok(Some(json!({"executed": true})))
        }),
    );

    // Register a condition evaluator
    planner.register_condition_evaluator(
        "battery_check",
        Arc::new(|condition: &TaskCondition, _ctx: &ExecutionContext| {
            // Extract args from Custom condition and check for "high"
            if let TaskCondition::Custom { name: _, args } = condition {
                args.contains(&"high".to_string())
            } else {
                false
            }
        }),
    );

    // Create tasks with different conditions
    let always_task = TaskSpec::new(
        "always",
        "Always runs",
        TaskExecutor::Custom {
            handler: "conditional_work".to_string(),
            parameters: HashMap::new(),
        },
    )
    .with_condition(TaskCondition::Always);

    let conditional_task = TaskSpec::new(
        "conditional",
        "Runs conditionally",
        TaskExecutor::Custom {
            handler: "conditional_work".to_string(),
            parameters: HashMap::new(),
        },
    )
    .with_condition(TaskCondition::Custom {
        name: "battery_check".to_string(),
        args: vec!["high".to_string()],
    });

    let never_task = TaskSpec::new(
        "never",
        "Never runs",
        TaskExecutor::Custom {
            handler: "conditional_work".to_string(),
            parameters: HashMap::new(),
        },
    )
    .with_condition(TaskCondition::Never)
    .optional(); // Mark as optional so unmet condition skips rather than fails

    let always_id = always_task.id.clone();
    let conditional_id = conditional_task.id.clone();

    let goal = GoalSpec::new("conditional", "Conditional goal")
        .add_task(always_task)
        .add_task_after(conditional_task, &always_id)
        .add_task_after(never_task, &conditional_id);

    let mission = MissionSpec::with_id("cond_mission", "Conditional mission").add_goal(goal);

    let id = planner.submit(mission).unwrap();
    planner.start(&id).unwrap();

    for _ in 0..20 {
        planner.tick().unwrap();
        let status = planner.get_mission_status(&id).unwrap();
        if status.status == ExecutionStatus::Completed {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }

    let status = planner.get_mission_status(&id).unwrap();
    assert_eq!(status.status, ExecutionStatus::Completed);

    // Check which tasks were executed
    let executed = executed_tasks.lock().unwrap();
    assert!(executed.contains(&"always".to_string()));
    assert!(executed.contains(&"conditional".to_string())); // Condition was met
    assert!(!executed.contains(&"never".to_string())); // Should not run
}

// =============================================================================
// Test 7: Multi-Goal Mission with Dependencies
// =============================================================================

/// Tests a complex mission with multiple goals that have dependencies.
#[test]
fn test_multi_goal_mission_with_dependencies() {
    let goal_order = Arc::new(Mutex::new(Vec::<String>::new()));

    let mut planner = MissionPlanner::new();

    let order = Arc::clone(&goal_order);
    planner.register_executor(
        "record_goal",
        Arc::new(move |task: &TaskSpec, _ctx: &ExecutionContext| {
            let params = get_executor_params(task);
            let goal_name = params
                .and_then(|p| p.get("goal_name"))
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            let mut o = order.lock().unwrap();
            o.push(goal_name.to_string());
            Ok(Some(json!({"recorded": goal_name})))
        }),
    );

    // Create goals with dependencies:
    // G1 (init) -> G2 (load) -> G3 (process)
    //           -> G4 (validate)
    // G2 and G4 can run in parallel after G1
    // G3 depends on G2 completing

    let goal1 = GoalSpec::new("init", "Initialize").add_task(TaskSpec::new(
        "init_task",
        "Init",
        TaskExecutor::Custom {
            handler: "record_goal".to_string(),
            parameters: json_to_hashmap(json!({"goal_name": "init"})),
        },
    ));

    let goal2 = GoalSpec::new("load", "Load data").add_task(TaskSpec::new(
        "load_task",
        "Load",
        TaskExecutor::Custom {
            handler: "record_goal".to_string(),
            parameters: json_to_hashmap(json!({"goal_name": "load"})),
        },
    ));

    let goal3 = GoalSpec::new("process", "Process data").add_task(TaskSpec::new(
        "process_task",
        "Process",
        TaskExecutor::Custom {
            handler: "record_goal".to_string(),
            parameters: json_to_hashmap(json!({"goal_name": "process"})),
        },
    ));

    let goal4 = GoalSpec::new("validate", "Validate").add_task(TaskSpec::new(
        "validate_task",
        "Validate",
        TaskExecutor::Custom {
            handler: "record_goal".to_string(),
            parameters: json_to_hashmap(json!({"goal_name": "validate"})),
        },
    ));

    let g1_id = goal1.id.clone();
    let g2_id = goal2.id.clone();

    let mission = MissionSpec::with_id("multi_goal", "Multi-goal mission")
        .add_goal(goal1)
        .add_goal_after(goal2, &g1_id)
        .add_goal_after(goal4, &g1_id)
        .add_goal_after(goal3, &g2_id);

    let id = planner.submit(mission).unwrap();
    planner.start(&id).unwrap();

    for _ in 0..50 {
        planner.tick().unwrap();
        let status = planner.get_mission_status(&id).unwrap();
        if status.status == ExecutionStatus::Completed {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }

    let status = planner.get_mission_status(&id).unwrap();
    assert_eq!(status.status, ExecutionStatus::Completed);

    // Verify order constraints
    let order = goal_order.lock().unwrap();
    assert_eq!(order.len(), 4);
    assert_eq!(order[0], "init"); // Init must be first

    // Find positions
    let load_pos = order.iter().position(|g| g == "load").unwrap();
    let process_pos = order.iter().position(|g| g == "process").unwrap();

    // Process must come after load
    assert!(process_pos > load_pos);
}

// =============================================================================
// Test 8: Robot State Machine Workflow
// =============================================================================

/// Tests a complete robot state machine with multiple modes.
#[test]
fn test_robot_state_machine_workflow() {
    struct RobotState {
        mode: String,
        sensor_data: f64,
        error_count: u32,
    }

    let fsm = StateMachineBuilder::<RobotState>::new("robot")
        .initial_state("startup")
        .on_entry(|ctx| ctx.mode = "startup".to_string())
        .done()
        .state("calibrating")
        .on_entry(|ctx| ctx.mode = "calibrating".to_string())
        .on_tick(|ctx| ctx.sensor_data += 0.1)
        .done()
        .state("ready")
        .on_entry(|ctx| ctx.mode = "ready".to_string())
        .done()
        .state("operating")
        .on_entry(|ctx| ctx.mode = "operating".to_string())
        .done()
        .state("fault")
        .on_entry(|ctx| {
            ctx.mode = "fault".to_string();
            ctx.error_count += 1;
        })
        .done()
        .state("shutdown")
        .on_entry(|ctx| ctx.mode = "shutdown".to_string())
        .done()
        .transition("startup", "calibrating")
        .on_event("init_complete")
        .done()
        .transition("calibrating", "ready")
        .on_event("calibrated")
        .with_guard(|ctx| ctx.sensor_data > 0.5)
        .done()
        .transition("ready", "operating")
        .on_event("start")
        .done()
        .transition("operating", "ready")
        .on_event("stop")
        .done()
        .transition("operating", "fault")
        .on_event("error")
        .done()
        .transition("fault", "ready")
        .on_event("recover")
        .done()
        .transition("ready", "shutdown")
        .on_event("shutdown")
        .done()
        .build()
        .unwrap();

    let mut ctx = RobotState {
        mode: String::new(),
        sensor_data: 0.0,
        error_count: 0,
    };

    let mut fsm = fsm;
    fsm.start(&mut ctx).unwrap();
    assert_eq!(ctx.mode, "startup");

    // Workflow: startup -> calibrating -> ready -> operating -> fault -> ready -> shutdown

    fsm.process_event(&Event::new("init_complete"), &mut ctx)
        .unwrap();
    assert_eq!(ctx.mode, "calibrating");

    // Simulate calibration ticks
    for _ in 0..10 {
        fsm.tick(&mut ctx).unwrap();
    }
    assert!(ctx.sensor_data > 0.5);

    fsm.process_event(&Event::new("calibrated"), &mut ctx)
        .unwrap();
    assert_eq!(ctx.mode, "ready");

    fsm.process_event(&Event::new("start"), &mut ctx).unwrap();
    assert_eq!(ctx.mode, "operating");

    fsm.process_event(&Event::new("error"), &mut ctx).unwrap();
    assert_eq!(ctx.mode, "fault");
    assert_eq!(ctx.error_count, 1);

    fsm.process_event(&Event::new("recover"), &mut ctx).unwrap();
    assert_eq!(ctx.mode, "ready");

    fsm.process_event(&Event::new("shutdown"), &mut ctx)
        .unwrap();
    assert_eq!(ctx.mode, "shutdown");
}

// =============================================================================
// Test 9: Mission Progress and Metrics
// =============================================================================

/// Tests mission progress tracking and metrics collection.
#[test]
fn test_mission_progress_and_metrics() {
    let progress_updates = Arc::new(Mutex::new(Vec::<f64>::new()));
    let task_events = Arc::new(Mutex::new(Vec::<String>::new()));

    let mut planner = MissionPlannerBuilder::new().collect_metrics(true).build();

    // Set up event callback
    let progress = Arc::clone(&progress_updates);
    let events = Arc::clone(&task_events);
    planner.on_event(Arc::new(move |event: &MissionEvent| match event {
        MissionEvent::Progress {
            progress: p,
            mission_id: _,
        } => {
            let mut prog = progress.lock().unwrap();
            prog.push(*p);
        }
        MissionEvent::TaskCompleted { task_id, .. } => {
            let mut ev = events.lock().unwrap();
            ev.push(format!("completed:{}", task_id.as_str()));
        }
        MissionEvent::TaskStarted { task_id, .. } => {
            let mut ev = events.lock().unwrap();
            ev.push(format!("started:{}", task_id.as_str()));
        }
        _ => {}
    }));

    // Create mission with multiple tasks
    let task1 = TaskSpec::noop("t1", "Task 1");
    let task2 = TaskSpec::noop("t2", "Task 2");
    let task3 = TaskSpec::noop("t3", "Task 3");
    let task4 = TaskSpec::noop("t4", "Task 4");

    let t1_id = task1.id.clone();
    let t2_id = task2.id.clone();
    let t3_id = task3.id.clone();

    let goal = GoalSpec::new("progress_goal", "Progress goal")
        .add_task(task1)
        .add_task_after(task2, &t1_id)
        .add_task_after(task3, &t2_id)
        .add_task_after(task4, &t3_id);

    let mission = MissionSpec::with_id("progress_mission", "Progress mission").add_goal(goal);

    let id = planner.submit(mission).unwrap();
    planner.start(&id).unwrap();

    for _ in 0..30 {
        planner.tick().unwrap();
        let status = planner.get_mission_status(&id).unwrap();
        if status.status == ExecutionStatus::Completed {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }

    // Check metrics
    let metrics = planner.metrics();
    assert!(metrics.total_missions >= 1);
    assert!(metrics.successful_missions >= 1);

    // Check events were recorded
    let events = task_events.lock().unwrap();
    assert!(events.len() >= 4); // At least 4 tasks completed
}

// =============================================================================
// Test 10: Concurrent Mission Execution
// =============================================================================

/// Tests running multiple missions concurrently.
#[test]
fn test_concurrent_mission_execution() {
    let completed_missions = Arc::new(AtomicUsize::new(0));

    let mut planner = MissionPlannerBuilder::new()
        .max_concurrent_missions(3)
        .max_concurrent_tasks(6)
        .build();

    let completed = Arc::clone(&completed_missions);
    planner.on_event(Arc::new(move |event: &MissionEvent| {
        if let MissionEvent::MissionCompleted { success, .. } = event {
            if *success {
                completed.fetch_add(1, Ordering::SeqCst);
            }
        }
    }));

    // Create 3 missions
    let missions: Vec<_> = (0..3)
        .map(|i| {
            let task = TaskSpec::noop(format!("task_{}", i), format!("Task {}", i));
            let goal = GoalSpec::new(format!("goal_{}", i), format!("Goal {}", i)).add_task(task);
            MissionSpec::with_id(format!("mission_{}", i), format!("Mission {}", i)).add_goal(goal)
        })
        .collect();

    // Submit all missions
    let ids: Vec<_> = missions
        .into_iter()
        .map(|m| planner.submit(m).unwrap())
        .collect();

    // Start all missions
    for id in &ids {
        planner.start(id).unwrap();
    }

    // Tick until all complete
    for _ in 0..50 {
        planner.tick().unwrap();
        let all_done = ids.iter().all(|id| {
            let status = planner.get_mission_status(id).unwrap();
            status.status == ExecutionStatus::Completed
        });
        if all_done {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }

    // Verify all completed
    for id in &ids {
        let status = planner.get_mission_status(id).unwrap();
        assert_eq!(status.status, ExecutionStatus::Completed);
    }

    assert_eq!(completed_missions.load(Ordering::SeqCst), 3);
}
