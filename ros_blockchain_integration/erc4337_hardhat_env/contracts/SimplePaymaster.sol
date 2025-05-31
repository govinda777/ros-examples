// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20; // Adjusted for compatibility

// For @account-abstraction/contracts v0.6.0, UserOperation is often imported directly from its interface file
import {UserOperation} from "@account-abstraction/contracts/interfaces/UserOperation.sol";
import "@account-abstraction/contracts/core/BasePaymaster.sol";
import "@account-abstraction/contracts/interfaces/IEntryPoint.sol";


contract SimplePaymaster is BasePaymaster {
    event Funded(address indexed sender, uint256 amount);

    constructor(IEntryPoint _entryPoint) BasePaymaster(_entryPoint) {}

    function deposit() public payable {
        entryPoint().depositTo(address(this), msg.value);
        emit Funded(msg.sender, msg.value);
    }

    function getDeposit() public view returns (uint256) {
        return entryPoint().balanceOf(address(this));
    }

    function _validatePaymasterUserOp(
        UserOperation calldata userOp, // Using UserOperation directly as per v0.6.0 BasePaymaster
        bytes32 userOpHash,
        uint256 maxCost
    ) internal view override returns (bytes memory context, uint256 validationData) {
        if (getDeposit() >= maxCost) {
            return (abi.encode(1), 0);
        }
        return ("", 0);
    }

    function _postOp(PostOpMode mode, bytes calldata context, uint256 actualGasCost) internal override {
        super._postOp(mode, context, actualGasCost);
        if (context.length == 0) {
            return;
        }
    }

    receive() external payable {
        deposit();
    }
}
