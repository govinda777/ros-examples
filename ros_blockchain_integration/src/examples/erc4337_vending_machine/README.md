# ERC-4337 Vending Machine Example (Simplified)

This example demonstrates a simulated vending machine interacting with a smart contract on a local Ethereum blockchain (Ganache). A ROS node simulates the vending machine's dispensing mechanism, and a client script simulates a user purchasing an item.

**Note on ERC-4337:** Due to environmental constraints during development, this example currently uses standard Ethereum transactions instead of full ERC-4337 UserOperations and Paymasters. The client script (`erc4337_vending_client.py`) contains placeholder comments where ERC-4337 specific logic (UserOperation creation, bundling, Paymaster interaction) would be implemented. The focus is on the overall flow of blockchain-triggered robotic action.

## Components

1.  **`VendingMachine.sol`**:
    *   Location: `ros_blockchain_integration/src/smart_contracts/VendingMachine.sol`
    *   A Solidity smart contract that manages items, prices, and handles purchase logic.
    *   Deployed Address (on local Ganache): `0x981197935cc110301Ccf86923Ea710Eec8549AF4` (This may change if Ganache is restarted or the contract is redeployed).
2.  **`deploy_vending_machine.py`**:
    *   Location: `ros_blockchain_integration/src/smart_contracts/deploy_vending_machine.py`
    *   A Python script to compile and deploy `VendingMachine.sol` to Ganache and add an initial item.
3.  **`vending_machine_node.py`**:
    *   Location: `ros_blockchain_integration/src/ros_nodes/vending_machine_node.py`
    *   A ROS node that subscribes to the `/dispense_item` topic. Upon receiving an item ID, it simulates dispensing the item.
4.  **`erc4337_vending_client.py`**:
    *   Location: `ros_blockchain_integration/src/examples/erc4337_vending_machine/erc4337_vending_client.py`
    *   A Python script that allows a user to "purchase" an item. It interacts with the `VendingMachine.sol` contract and then publishes the item ID to the `/dispense_item` ROS topic.

## Prerequisites

*   ROS (Robot Operating System) installed (e.g., Noetic or a version compatible with Python 3 for the ROS nodes).
*   Docker installed and running.
*   Python 3 and pip.
*   Required Python packages: `web3`, `py-solc-x`, `rospy` (usually part of ROS installation). Install with:
    ```bash
    pip install web3 py-solc-x
    ```

## Setup and Running the Example

1.  **Start Ganache Blockchain:**
    *   Ensure Docker is running.
    *   In a terminal, start the Ganache container (if not already running from previous steps):
        ```bash
        sudo docker run -d -p 8545:8545 trufflesuite/ganache:latest
        ```
    *   Verify it's running: `sudo docker ps` and `curl -X POST --data '{"jsonrpc":"2.0","method":"eth_blockNumber","params":[],"id":1}' -H "Content-Type: application/json" http://localhost:8545`

2.  **Deploy Smart Contract:**
    *   Navigate to the root of the `ros_blockchain_integration` project (e.g., `/app` in this environment).
    *   Run the deployment script:
        ```bash
        python3 ros_blockchain_integration/src/smart_contracts/deploy_vending_machine.py
        ```
    *   Note the deployed `VendingMachine` contract address from the script's output. If it's different from the one hardcoded in `erc4337_vending_client.py`, update the client script. (The current hardcoded address is `0x981197935cc110301Ccf86923Ea710Eec8549AF4`).

3.  **Start ROS Master and Vending Machine Node:**
    *   In a new terminal, source your ROS environment and start `roscore`:
        ```bash
        # Example for ROS Noetic, adjust to your ROS distribution
        source /opt/ros/noetic/setup.bash
        # Or your workspace setup.bash if you have one: source devel/setup.bash
        roscore
        ```
    *   In another terminal, source your ROS environment and run the vending machine ROS node:
        ```bash
        # Example for ROS Noetic
        source /opt/ros/noetic/setup.bash
        # Or your workspace setup.bash
        python3 ros_blockchain_integration/src/ros_nodes/vending_machine_node.py
        ```
    *   You should see log messages indicating the node is initialized.

4.  **Run the Client Script:**
    *   In another terminal (ensure Python 3 environment with `web3` is active, and ROS environment is sourced):
        ```bash
        # Example for ROS Noetic
        source /opt/ros/noetic/setup.bash
        # Or your workspace setup.bash
        python3 ros_blockchain_integration/src/examples/erc4337_vending_machine/erc4337_vending_client.py
        ```
    *   The script will connect to Ganache, show item details, and prompt if you want to purchase "RoboCola".
    *   If you confirm, it will send a transaction. Upon confirmation, it will publish to the `/dispense_item` ROS topic.
    *   You should see the `vending_machine_node.py` terminal log that it received the command and is "dispensing" the item.

## Configuration

*   **Ganache RPC URL:** `http://localhost:8545` (hardcoded in `erc4337_vending_client.py` and `deploy_vending_machine.py`).
*   **Vending Machine Contract Address:** Currently hardcoded as `0x981197935cc110301Ccf86923Ea710Eec8549AF4`. Update in `erc4337_vending_client.py` if redeployment results in a new address. The `deploy_vending_machine.py` script will output the deployed address.
*   **Item ID for Purchase:** The client script is currently hardcoded to attempt purchasing item ID 1 ("RoboCola").

## Future ERC-4337 Integration Points
*   Modify `erc4337_vending_client.py` to construct and send `UserOperation`s via a Bundler.
*   Implement and deploy an ERC-4337 `EntryPoint` and `Paymaster` contract (e.g., using the Hardhat environment set up in previous steps, if successfully completed).
*   Update `VendingMachine.sol` to potentially have functions callable only by the `EntryPoint` or to interact with a Paymaster for sponsored transactions.
```
