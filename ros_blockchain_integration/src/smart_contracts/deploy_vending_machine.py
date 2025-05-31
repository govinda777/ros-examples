import json
import solcx
from web3 import Web3
import os

# Configuration
GANACHE_URL = "http://localhost:8545"
SOLIDITY_FILE_PATH = "ros_blockchain_integration/src/smart_contracts/VendingMachine.sol"
CONTRACT_NAME = "VendingMachine"

def install_solcx_version_if_needed(solidity_version):
    """Installs the specified solc version if not already installed."""
    try:
        installed_versions = solcx.get_installed_solc_versions()
        if solidity_version not in installed_versions:
            print(f"solc version {solidity_version} not found. Installing...")
            solcx.install_solc(solidity_version)
            print(f"solc version {solidity_version} installed successfully.")
        else:
            print(f"solc version {solidity_version} is already installed.")
        solcx.set_solc_version(solidity_version)
    except Exception as e:
        print(f"Error managing solcx versions: {e}")
        print("Please ensure solc is installed and accessible, or try installing a specific version manually.")
        raise

def compile_source_file(file_path, contract_name):
    """Compiles the Solidity contract and returns ABI and bytecode."""
    try:
        with open(file_path, 'r') as file:
            source_code = file.read()

        # Determine Solidity version from pragma (simple parsing)
        pragma_line = next((line for line in source_code.splitlines() if "pragma solidity" in line), None)
        if not pragma_line:
            raise ValueError("No pragma solidity line found in the contract.")

        # Extract version, e.g., from "^0.8.0" or "0.8.0"
        version_str = pragma_line.split("solidity")[1].replace("^", "").replace(";", "").strip().split('.')[0] + "." + \
                      pragma_line.split("solidity")[1].replace("^", "").replace(";", "").strip().split('.')[1] + "." + \
                      pragma_line.split("solidity")[1].replace("^", "").replace(";", "").strip().split('.')[2].split(';')[0].split(' ')[0]


        # A more robust way to get the version part, e.g. 0.8.0 from ^0.8.0
        if ">=" in version_str:
            solidity_version = version_str.split(">=")[1].strip().split("<")[0].strip()
        elif ">" in version_str:
            solidity_version = version_str.split(">")[1].strip().split("<")[0].strip()
        else: # Handles exact versions or caret versions like ^0.8.0
             solidity_version = version_str


        # Ensure major.minor.patch format, assuming 0.8.0 if only 0.8 is found (adjust if needed)
        parts = solidity_version.split('.')
        if len(parts) == 2: # e.g. 0.8
            solidity_version += '.0'
        elif len(parts) < 2:
             raise ValueError(f"Could not determine a valid Solidity version from pragma: {pragma_line}")


        print(f"Detected Solidity version: {solidity_version} from pragma: {pragma_line}")
        install_solcx_version_if_needed(solidity_version)

        compiled_sol = solcx.compile_source(
            source_code,
            output_values=['abi', 'bin'],
            solc_version=solidity_version
        )

        contract_interface = compiled_sol.get(f'<stdin>:{contract_name}')
        if not contract_interface:
            raise ValueError(f"Contract {contract_name} not found in compiled output. Available keys: {compiled_sol.keys()}")

        return contract_interface['abi'], contract_interface['bin']
    except solcx.exceptions.SolcError as e:
        print(f"Solidity compilation error: {e}")
        raise
    except Exception as e:
        print(f"Error during compilation: {e}")
        raise

def main():
    print("Starting VendingMachine contract deployment...")

    # Ensure solcx has a version set (e.g. latest compatible or from contract)
    # For this example, we'll extract from contract, but you might set a default
    # install_solcx_version_if_needed("0.8.19") # Example, will be replaced by dynamic one

    try:
        # Compile the contract
        print(f"Compiling {SOLIDITY_FILE_PATH}...")
        abi, bytecode = compile_source_file(SOLIDITY_FILE_PATH, CONTRACT_NAME)
        print("Contract compiled successfully.")

        # Connect to Ganache
        w3 = Web3(Web3.HTTPProvider(GANACHE_URL))
        if not w3.is_connected():
            print(f"Failed to connect to Ganache at {GANACHE_URL}")
            return
        print(f"Connected to Ganache: {GANACHE_URL}")

        # Set default account (deployer)
        w3.eth.default_account = w3.eth.accounts[0]
        deployer_account = w3.eth.default_account
        print(f"Using deployer account: {deployer_account}")

        # Deploy the contract
        VendingMachine = w3.eth.contract(abi=abi, bytecode=bytecode)
        print("Deploying VendingMachine contract...")
        tx_hash = VendingMachine.constructor().transact()
        print(f"Deployment transaction sent, hash: {tx_hash.hex()}")
        tx_receipt = w3.eth.wait_for_transaction_receipt(tx_hash)
        contract_address = tx_receipt.contractAddress
        print(f"VendingMachine contract deployed at: {contract_address}")

        # Interact with the deployed contract: Add an item
        vending_machine_instance = w3.eth.contract(address=contract_address, abi=abi)

        item_name = "RoboCola"
        item_price_eth = 0.01
        item_price_wei = w3.to_wei(item_price_eth, 'ether')

        print(f"Adding item '{item_name}' with price {item_price_eth} ETH ({item_price_wei} wei)...")
        add_item_tx_hash = vending_machine_instance.functions.addItem(item_name, item_price_wei).transact({
            'from': deployer_account
        })
        print(f"addItem transaction sent, hash: {add_item_tx_hash.hex()}")
        add_item_tx_receipt = w3.eth.wait_for_transaction_receipt(add_item_tx_hash)

        if add_item_tx_receipt.status == 1:
            print(f"Item '{item_name}' added successfully. Transaction status: Success")
            # Verify by trying to get the item (optional)
            item_id_to_check = 1 # Assuming this is the first item added
            try:
                item_details = vending_machine_instance.functions.getItem(item_id_to_check).call()
                print(f"Verification: Item ID {item_id_to_check} - Name: {item_details[1]}, Price: {item_details[2]} wei, Active: {item_details[3]}")
            except Exception as e:
                print(f"Could not verify item addition: {e}")
        else:
            print(f"Failed to add item '{item_name}'. Transaction status: Failed")

        print("\n--- Deployment Summary ---")
        print(f"VendingMachine Contract Address: {contract_address}")
        print(f"addItem Transaction Hash: {add_item_tx_hash.hex()}")

    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    # Install web3 and py-solc-x if not present (basic check)
    try:
        import web3
        import solcx
    except ImportError:
        print("web3 or py-solc-x not found. Attempting to install...")
        os.system("pip install web3 py-solc-x")
        print("Please re-run the script after installation.")
        exit()
    main()
