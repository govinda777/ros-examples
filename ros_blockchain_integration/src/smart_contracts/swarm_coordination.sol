// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract SwarmCoordination {
    struct DroneInfo {
        address droneAddress;
        uint256 batteryLevel;
        bool isAvailable;
        uint256 lastUpdate;
        string droneId; // Added for easier identification
    }

    struct Task {
        uint256 taskId;
        string taskDescription;
        uint256 reward; // in wei
        address assignedDrone;
        address taskCreator;
        bool completed;
        bool cancelled;
        uint256 deadline; // Unix timestamp
        string taskDataCID; // IPFS Content ID for larger task data
    }

    mapping(address => DroneInfo) public drones;
    mapping(uint256 => Task) public tasks;

    uint256 public nextTaskId = 1;
    address public contractOwner;

    event DroneRegistered(address indexed drone, string droneId, uint256 batteryLevel);
    event DroneDeRegistered(address indexed drone, string droneId);
    event DroneStatusUpdated(address indexed drone, uint256 batteryLevel, bool isAvailable);
    event TaskCreated(uint256 indexed taskId, address indexed creator, uint256 reward, uint256 deadline, string taskDataCID);
    event TaskAssigned(uint256 indexed taskId, address indexed drone);
    event TaskAssignmentFailed(uint256 indexed taskId, address indexed drone, string reason);
    event TaskCompleted(uint256 indexed taskId, address indexed drone);
    event TaskCancelled(uint256 indexed taskId);
    event RewardPaid(uint256 indexed taskId, address indexed drone, uint256 amount);

    modifier onlyContractOwner() {
        require(msg.sender == contractOwner, "Caller is not the contract owner");
        _;
    }

    modifier onlyTaskCreator(uint256 _taskId) {
        require(tasks[_taskId].taskCreator == msg.sender, "Caller is not the task creator");
        _;
    }

    modifier droneIsRegistered() {
        require(drones[msg.sender].droneAddress != address(0), "Drone not registered");
        _;
    }

    modifier taskExists(uint256 _taskId) {
        require(tasks[_taskId].taskId != 0, "Task does not exist");
        _;
    }

    constructor() {
        contractOwner = msg.sender;
    }

    function registerDrone(string memory _droneId, uint256 _batteryLevel) external {
        require(drones[msg.sender].droneAddress == address(0), "Drone already registered with this address");
        // Consider adding a check to ensure _droneId is unique if required by the system
        drones[msg.sender] = DroneInfo({
            droneAddress: msg.sender,
            batteryLevel: _batteryLevel,
            isAvailable: true,
            lastUpdate: block.timestamp,
            droneId: _droneId
        });

        emit DroneRegistered(msg.sender, _droneId, _batteryLevel);
    }

    function deRegisterDrone() external droneIsRegistered {
        // Consider implications: what happens to assigned tasks?
        // For now, simple de-registration.
        string memory droneId = drones[msg.sender].droneId;
        delete drones[msg.sender];
        emit DroneDeRegistered(msg.sender, droneId);
    }

    function createTask(
        string memory _description,
        uint256 _deadline, // Unix timestamp
        string memory _taskDataCID // IPFS CID for large data
    ) external payable {
        require(msg.value > 0, "Task must have a reward");
        require(_deadline > block.timestamp, "Deadline must be in the future");

        tasks[nextTaskId] = Task({
            taskId: nextTaskId,
            taskDescription: _description,
            reward: msg.value,
            assignedDrone: address(0),
            taskCreator: msg.sender,
            completed: false,
            cancelled: false,
            deadline: _deadline,
            taskDataCID: _taskDataCID
        });

        emit TaskCreated(nextTaskId, msg.sender, msg.value, _deadline, _taskDataCID);
        nextTaskId++;
    }

    function bidForTask(uint256 _taskId) external droneIsRegistered taskExists(_taskId) {
        Task storage currentTask = tasks[_taskId];
        DroneInfo storage drone = drones[msg.sender];

        require(drone.isAvailable, "Drone not available");
        require(currentTask.assignedDrone == address(0), "Task already assigned");
        require(!currentTask.completed, "Task already completed");
        require(!currentTask.cancelled, "Task has been cancelled");
        require(block.timestamp < currentTask.deadline, "Task deadline passed");

        // Example condition: battery > 30% (adjust as needed)
        if (drone.batteryLevel > 30) {
            currentTask.assignedDrone = msg.sender;
            drone.isAvailable = false;
            drone.lastUpdate = block.timestamp;

            emit TaskAssigned(_taskId, msg.sender);
        } else {
            emit TaskAssignmentFailed(_taskId, msg.sender, "Insufficient battery level");
        }
    }

    function completeTask(uint256 _taskId) external droneIsRegistered taskExists(_taskId) {
        Task storage currentTask = tasks[_taskId];
        DroneInfo storage drone = drones[msg.sender];

        require(currentTask.assignedDrone == msg.sender, "Task not assigned to this drone");
        require(!currentTask.completed, "Task already completed");
        require(!currentTask.cancelled, "Task has been cancelled");

        currentTask.completed = true;
        drone.isAvailable = true; // Drone becomes available again
        drone.lastUpdate = block.timestamp;

        // Transfer reward
        // Ensure contract has enough balance (though reward was sent at creation)
        payable(msg.sender).transfer(currentTask.reward);

        emit TaskCompleted(_taskId, msg.sender);
        emit RewardPaid(_taskId, msg.sender, currentTask.reward);
    }

    function cancelTask(uint256 _taskId) external taskExists(_taskId) onlyTaskCreator(_taskId) {
        Task storage currentTask = tasks[_taskId];
        require(!currentTask.completed, "Cannot cancel a completed task");
        require(!currentTask.cancelled, "Task already cancelled");

        // Refund logic
        if (currentTask.assignedDrone == address(0)) {
            // No drone assigned, refund creator fully
            payable(currentTask.taskCreator).transfer(currentTask.reward);
        } else {
            // Task was assigned. Partial refund or other logic might be needed.
            // For simplicity, refunding fully if not completed.
            // In a real scenario, might penalize cancellation or have dispute resolution.
            payable(currentTask.taskCreator).transfer(currentTask.reward);

            // Make the assigned drone available again
            DroneInfo storage drone = drones[currentTask.assignedDrone];
            drone.isAvailable = true;
            drone.lastUpdate = block.timestamp;
            emit DroneStatusUpdated(currentTask.assignedDrone, drone.batteryLevel, drone.isAvailable);
        }

        currentTask.cancelled = true;
        currentTask.reward = 0; // Zero out reward after refund
        emit TaskCancelled(_taskId);
    }

    function updateDroneStatus(uint256 _newBatteryLevel, bool _isAvailable) external droneIsRegistered {
        DroneInfo storage drone = drones[msg.sender];
        drone.batteryLevel = _newBatteryLevel;
        drone.isAvailable = _isAvailable;
        drone.lastUpdate = block.timestamp;
        emit DroneStatusUpdated(msg.sender, _newBatteryLevel, _isAvailable);
    }

    // Getter functions
    function getDroneInfo(address _droneAddress) external view returns (string memory, uint256, bool, uint256) {
        DroneInfo storage drone = drones[_droneAddress];
        require(drone.droneAddress != address(0), "Drone not registered");
        return (drone.droneId, drone.batteryLevel, drone.isAvailable, drone.lastUpdate);
    }

    function getTaskDetails(uint256 _taskId) external view taskExists(_taskId) returns (string memory, uint256, address, address, bool, bool, uint256, string memory) {
        Task storage task = tasks[_taskId];
        return (task.taskDescription, task.reward, task.assignedDrone, task.taskCreator, task.completed, task.cancelled, task.deadline, task.taskDataCID);
    }

    // Fallback function to receive Ether (e.g., if someone sends Ether directly to contract for other purposes)
    receive() external payable {}

    // Function to allow owner to withdraw any Ether sent to the contract (not task rewards)
    function withdrawContractBalance() external onlyContractOwner {
        payable(contractOwner).transfer(address(this).balance);
    }
}
