// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract VendingMachine {
    address public owner;

    struct Item {
        uint id;
        string name;
        uint price; // in wei
        bool isActive;
    }

    mapping(uint => Item) public items;
    uint public nextItemId;

    event ItemAdded(uint id, string name, uint price);
    event ItemPurchased(address indexed buyer, uint indexed itemId, string name, uint price);
    event FundsWithdrawn(address indexed to, uint amount);

    modifier onlyOwner() {
        require(msg.sender == owner, "VendingMachine: Caller is not the owner");
        _;
    }

    constructor() {
        owner = msg.sender;
        nextItemId = 1; // Start item IDs from 1
    }

    function addItem(string memory _name, uint _price) external onlyOwner {
        items[nextItemId] = Item(nextItemId, _name, _price, true);
        emit ItemAdded(nextItemId, _name, _price);
        nextItemId++;
    }

    function getItem(uint _itemId) external view returns (uint id, string memory name, uint price, bool isActive) {
        Item storage item = items[_itemId];
        require(item.id != 0, "VendingMachine: Item does not exist");
        return (item.id, item.name, item.price, item.isActive);
    }

    function purchaseItem(uint _itemId) external payable {
        Item storage item = items[_itemId];
        require(item.id != 0, "VendingMachine: Item does not exist");
        require(item.isActive, "VendingMachine: Item is not active");
        require(msg.value == item.price, "VendingMachine: Incorrect payment amount");

        // In a real scenario, you might check inventory or interact with a dispensing mechanism.
        // For this example, emitting an event is sufficient.
        emit ItemPurchased(msg.sender, _itemId, item.name, item.price);
    }

    function withdrawFunds() external onlyOwner {
        uint balance = address(this).balance;
        require(balance > 0, "VendingMachine: No funds to withdraw");
        payable(owner).transfer(balance);
        emit FundsWithdrawn(owner, balance);
    }

    // Fallback function to receive ETH if sent directly (e.g. for funding paymaster later if we integrate)
    receive() external payable {}
}
