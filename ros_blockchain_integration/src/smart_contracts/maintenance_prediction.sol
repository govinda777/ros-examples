// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

// Smart contract for predictive maintenance based on sensor data.
// Implementation will be added later.
contract MaintenancePrediction {
    struct MaintenanceRecord {
        uint256 deviceId;
        uint256 lastMaintenanceTimestamp;
        uint256 predictedFailureTimestamp;
        string maintenanceNotes;
    }

    mapping(uint256 => MaintenanceRecord) public maintenanceSchedule;
    address public oracleAddress; // Address authorized to update predictions

    event PredictionUpdated(uint256 indexed deviceId, uint256 predictedFailureTimestamp);
    event MaintenanceScheduled(uint256 indexed deviceId, string notes);

    modifier onlyOracle() {
        require(msg.sender == oracleAddress, "Caller is not the authorized oracle");
        _;
    }

    constructor(address _oracleAddress) {
        oracleAddress = _oracleAddress;
    }

    function updateMaintenancePrediction(uint256 _deviceId, uint256 _predictedFailureTimestamp) public onlyOracle {
        maintenanceSchedule[_deviceId].predictedFailureTimestamp = _predictedFailureTimestamp;
        maintenanceSchedule[_deviceId].lastMaintenanceTimestamp = block.timestamp; // Assuming update implies recent check
        emit PredictionUpdated(_deviceId, _predictedFailureTimestamp);
    }

    function scheduleMaintenance(uint256 _deviceId, string memory _notes) public {
        // This function could be restricted, e.g., only by device owner or admin
        maintenanceSchedule[_deviceId].maintenanceNotes = _notes;
        // Potentially, interact with an external system to actually schedule the maintenance
        emit MaintenanceScheduled(_deviceId, _notes);
    }

    function getMaintenanceInfo(uint256 _deviceId) public view returns (uint256, uint256, string memory) {
        MaintenanceRecord storage record = maintenanceSchedule[_deviceId];
        return (record.lastMaintenanceTimestamp, record.predictedFailureTimestamp, record.maintenanceNotes);
    }
}
