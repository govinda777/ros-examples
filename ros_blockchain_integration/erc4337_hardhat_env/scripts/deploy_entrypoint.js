const hre = require("hardhat");

async function main() {
  const EntryPoint = await hre.ethers.getContractFactory("EntryPoint");
  const entryPoint = await EntryPoint.deploy();

  await entryPoint.waitForDeployment(); // Recommended to ensure contract is mined

  const entryPointAddress = await entryPoint.getAddress();
  console.log(`EntryPoint deployed to: ${entryPointAddress}`);

  // You can return the address if you need to use it in other scripts or tests programmatically
  return entryPointAddress;
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
