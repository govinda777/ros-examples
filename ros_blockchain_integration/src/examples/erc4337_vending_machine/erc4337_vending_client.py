#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt32
from web3 import Web3
import json
import time
import os # For potential pip install

# --- Configuration ---
GANACHE_URL = "http://localhost:8545"
# This is the address from the deployment in the previous subtask (VendingMachine contract)
# Make sure this address is correct and the contract is deployed on the target Ganache instance.
VENDING_MACHINE_CONTRACT_ADDRESS = "0x981197935cc110301Ccf86923Ea710Eec8549AF4"
# ABI (Application Binary Interface) for the VendingMachine smart contract.
# This JSON string defines the contract's functions, events, and structures,
# allowing web3.py to interact with it.
# IMPORTANT: This ABI must exactly match the compiled VendingMachine.sol.
# It can be obtained from compilation artifacts (e.g., if using Hardhat, it's in
# artifacts/contracts/VendingMachine.sol/VendingMachine.json) or from the
# output of tools like `solcx` used in `deploy_vending_machine.py`.
VENDING_MACHINE_ABI_STRING = '''
[
    {
        "inputs": [],
        "stateMutability": "nonpayable",
        "type": "constructor"
    },
    {
        "anonymous": false,
        "inputs": [
            {
                "indexed": false,
                "internalType": "uint256",
                "name": "id",
                "type": "uint256"
            },
            {
                "indexed": false,
                "internalType": "string",
                "name": "name",
                "type": "string"
            },
            {
                "indexed": false,
                "internalType": "uint256",
                "name": "price",
                "type": "uint256"
            }
        ],
        "name": "ItemAdded",
        "type": "event"
    },
    {
        "anonymous": false,
        "inputs": [
            {
                "indexed": true,
                "internalType": "address",
                "name": "buyer",
                "type": "address"
            },
            {
                "indexed": true,
                "internalType": "uint256",
                "name": "itemId",
                "type": "uint256"
            },
            {
                "indexed": false,
                "internalType": "string",
                "name": "name",
                "type": "string"
            },
            {
                "indexed": false,
                "internalType": "uint256",
                "name": "price",
                "type": "uint256"
            }
        ],
        "name": "ItemPurchased",
        "type": "event"
    },
    {
        "anonymous": false,
        "inputs": [
            {
                "indexed": true,
                "internalType": "address",
                "name": "to",
                "type": "address"
            },
            {
                "indexed": false,
                "internalType": "uint256",
                "name": "amount",
                "type": "uint256"
            }
        ],
        "name": "FundsWithdrawn",
        "type": "event"
    },
    {
        "inputs": [
            {
                "internalType": "string",
                "name": "_name",
                "type": "string"
            },
            {
                "internalType": "uint256",
                "name": "_price",
                "type": "uint256"
            }
        ],
        "name": "addItem",
        "outputs": [],
        "stateMutability": "nonpayable",
        "type": "function"
    },
    {
        "inputs": [
            {
                "internalType": "uint256",
                "name": "_itemId",
                "type": "uint256"
            }
        ],
        "name": "getItem",
        "outputs": [
            {
                "internalType": "uint256",
                "name": "id",
                "type": "uint256"
            },
            {
                "internalType": "string",
                "name": "name",
                "type": "string"
            },
            {
                "internalType": "uint256",
                "name": "price",
                "type": "uint256"
            },
            {
                "internalType": "bool",
                "name": "isActive",
                "type": "bool"
            }
        ],
        "stateMutability": "view",
        "type": "function"
    },
    {
        "inputs": [
            {
                "internalType": "uint256",
                "name": "",
                "type": "uint256"
            }
        ],
        "name": "items",
        "outputs": [
            {
                "internalType": "uint256",
                "name": "id",
                "type": "uint256"
            },
            {
                "internalType": "string",
                "name": "name",
                "type": "string"
            },
            {
                "internalType": "uint256",
                "name": "price",
                "type": "uint256"
            },
            {
                "internalType": "bool",
                "name": "isActive",
                "type": "bool"
            }
        ],
        "stateMutability": "view",
        "type": "function"
    },
    {
        "inputs": [],
        "name": "nextItemId",
        "outputs": [
            {
                "internalType": "uint256",
                "name": "",
                "type": "uint256"
            }
        ],
        "stateMutability": "view",
        "type": "function"
    },
    {
        "inputs": [],
        "name": "owner",
        "outputs": [
            {
                "internalType": "address",
                "name": "",
                "type": "address"
            }
        ],
        "stateMutability": "view",
        "type": "function"
    },
    {
        "inputs": [
            {
                "internalType": "uint256",
                "name": "_itemId",
                "type": "uint256"
            }
        ],
        "name": "purchaseItem",
        "outputs": [],
        "stateMutability": "payable",
        "type": "function"
    },
    {
        "inputs": [],
        "name": "withdrawFunds",
        "outputs": [],
        "stateMutability": "nonpayable",
        "type": "function"
    },
    {
        "stateMutability": "payable",
        "type": "receive"
    }
]'''
VENDING_MACHINE_ABI = json.loads(VENDING_MACHINE_ABI_STRING)

# ROS Publisher for dispensing items
ros_dispense_publisher = None
ros_node_initialized = False # Flag to track if rospy.init_node was called

def ensure_web3_installed():
    try:
        import web3
    except ImportError:
        print("web3 not found. Attempting to install...")
        try:
            # It's better to instruct user or handle this in setup, but for robustness:
            result = os.system("python3 -m pip install web3")
            if result != 0:
                raise Exception("pip install web3 failed")
            print("web3 installed successfully. Please re-run the script if it doesn't continue automatically.")
            # Re-check import to confirm
            import web3
        except Exception as e:
            print(f"Failed to install web3: {e}. Please install it manually: pip install web3")
            exit(1) # Exit if essential dependency is missing

def setup_ros():
    global ros_dispense_publisher, ros_node_initialized
    if ros_node_initialized: # Prevent re-initializing node if called multiple times
        return True
    try:
        rospy.init_node('erc4337_vending_client', anonymous=True)
        ros_node_initialized = True
        ros_dispense_publisher = rospy.Publisher('/dispense_item', UInt32, queue_size=10)
        rospy.loginfo("ROS components initialized for Vending Client.")
        return True
    except rospy.ROSInitException as e:
        print(f"Could not initialize ROS (roscore not running or ROS not sourced?): {e}. Running without ROS integration.")
    except Exception as e:
        print(f"An unexpected error occurred during ROS initialization: {e}. Running without ROS integration.")
    return False

def purchase_item_on_blockchain(w3, contract, item_id, buyer_account, item_price_wei):
    try:
        print(f"Attempting to purchase item ID {item_id} for {item_price_wei} wei from account {buyer_account}...")

        # Standard transaction (not ERC-4337 for this step)
        tx_hash = contract.functions.purchaseItem(item_id).transact({
            'from': buyer_account,
            'value': item_price_wei,
            'gas': 200000 # Adjusted gas limit for safety, can be estimated better
        })

        print(f"Transaction sent, waiting for receipt... TX Hash: {tx_hash.hex()}")
        tx_receipt = w3.eth.wait_for_transaction_receipt(tx_hash, timeout=120)

        if tx_receipt.status == 1:
            print("Purchase successful on blockchain!")
            logs = contract.events.ItemPurchased().process_receipt(tx_receipt)
            if logs:
                print(f"ItemPurchased event detected: {logs[0]['args']}")
                return True, logs[0]['args']
            else:
                print("Purchase seemed successful, but ItemPurchased event not found in receipt.")
                return True, None
            return True, None
        else:
            print(f"Purchase transaction failed. Receipt: {tx_receipt}")
            return False, None
    except Exception as e:
        print(f"Error purchasing item on blockchain: {e}")
        import traceback
        traceback.print_exc()
        return False, None

def main_client_logic():
    ros_active = setup_ros()

    # Ensure VENDING_MACHINE_CONTRACT_ADDRESS is valid
    if not VENDING_MACHINE_CONTRACT_ADDRESS or not Web3.is_address(VENDING_MACHINE_CONTRACT_ADDRESS):
        print(f"Error: Vending Machine Contract Address '{VENDING_MACHINE_CONTRACT_ADDRESS}' is not a valid Ethereum address.")
        print("Please deploy the VendingMachine contract first and update this script with its address.")
        return

    w3 = Web3(Web3.HTTPProvider(GANACHE_URL))
    if not w3.is_connected():
        print(f"Failed to connect to Ganache at {GANACHE_URL}")
        return

    print(f"Connected to Ethereum node (Ganache): {GANACHE_URL}") # Simpler version string

    try:
        # Using w3.eth.accounts[1] as the buyer for simplicity with Ganache.
        # Ganache pre-funds several accounts by default.
        # In a real application, account management (private keys, wallets, etc.)
        # would be handled securely and differently (e.g., using environment variables,
        # a keystore, or a hardware wallet, and not hardcoding private keys).
        # For ERC-4337, this would be the smart account address or an EOA controlling it.
        buyer_account = w3.eth.accounts[1]
    except IndexError:
        print("Error: Could not get account w3.eth.accounts[1]. Ensure Ganache is running and has accounts.")
        return

    print(f"Using buyer account: {buyer_account}")
    balance = w3.eth.get_balance(buyer_account)
    print(f"Buyer account balance: {w3.from_wei(balance, 'ether')} ETH")

    vending_machine_contract = w3.eth.contract(
        address=VENDING_MACHINE_CONTRACT_ADDRESS,
        abi=VENDING_MACHINE_ABI
    )

    item_id_to_purchase = 1
    try:
        print(f"Fetching details for item ID: {item_id_to_purchase}...")
        item_info = vending_machine_contract.functions.getItem(item_id_to_purchase).call()
        item_name = item_info[1]
        item_price_wei = item_info[2]
        item_is_active = item_info[3]

        if not item_is_active:
            print(f"Item '{item_name}' (ID: {item_id_to_purchase}) is not active. Cannot purchase.")
            return

        print(f"Selected item: ID {item_id_to_purchase}, Name: {item_name}, Price: {w3.from_wei(item_price_wei, 'ether')} ETH ({item_price_wei} wei)")

        if balance < item_price_wei:
            print(f"Buyer has insufficient funds ({w3.from_wei(balance, 'ether')} ETH) to buy {item_name} ({w3.from_wei(item_price_wei, 'ether')} ETH).")
            return

        user_input = input(f"Do you want to purchase '{item_name}' for {w3.from_wei(item_price_wei, 'ether')} ETH? (yes/no): ").lower()
        if user_input != 'yes':
            print("Purchase cancelled by user.")
            return

        print(f"User wants to buy item {item_name} (ID: {item_id_to_purchase}).")

        purchase_success, event_args = purchase_item_on_blockchain(
            w3, vending_machine_contract, item_id_to_purchase, buyer_account, item_price_wei
        )

        if purchase_success:
            print(f"Successfully processed purchase of '{item_name}' on-chain.")
            if ros_active and ros_dispense_publisher:
                if rospy.is_shutdown():
                    print("ROS has shut down before dispense message could be published.")
                    return
                print(f"Publishing dispense command to ROS topic /dispense_item for item ID {item_id_to_purchase}")
                dispense_msg = UInt32()
                dispense_msg.data = item_id_to_purchase
                ros_dispense_publisher.publish(dispense_msg)
                rospy.loginfo(f"Dispense command for item {item_id_to_purchase} published.")
                time.sleep(1) # Give ROS a moment
            else:
                print("ROS not active or publisher not available. Dispense command not sent via ROS.")
        else:
            print(f"Failed to process purchase of '{item_name}' on-chain.")

    except Exception as e:
        print(f"An error occurred in the main client loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if ros_active and not rospy.is_shutdown():
            rospy.loginfo("Vending client shutting down ROS components.")
            # rospy.signal_shutdown("Client finished.") # This might be too abrupt if other ROS parts are running

if __name__ == '__main__':
    ensure_web3_installed() # Ensure web3 is available before other imports might fail
    main_client_logic()
