const hre = require("hardhat");
const ethers = hre.ethers; // Alias for ethers

async function main() {
  // !!! IMPORTANT !!!
  // Replace this with the actual EntryPoint address obtained after deploying it.
  const entryPointAddress = "ENTRYPOINT_ADDRESS_PLACEHOLDER";

  if (entryPointAddress === "ENTRYPOINT_ADDRESS_PLACEHOLDER" || !ethers.isAddress(entryPointAddress)) {
    console.error("Please replace ENTRYPOINT_ADDRESS_PLACEHOLDER in scripts/deploy_paymaster.js with the actual deployed EntryPoint address.");
    process.exit(1);
  }

  const [deployer] = await ethers.getSigners();
  console.log("Deploying SimplePaymaster with account:", deployer.address);

  const Paymaster = await ethers.getContractFactory("SimplePaymaster");
  const paymaster = await Paymaster.deploy(entryPointAddress);

  await paymaster.waitForDeployment();
  const paymasterAddress = await paymaster.getAddress();
  console.log(`SimplePaymaster deployed to: ${paymasterAddress}`);

  // Fund the Paymaster
  const fundingAmount = ethers.parseEther("0.1"); // Fund with 0.1 ETH
  console.log(`Funding Paymaster with ${ethers.formatEther(fundingAmount)} ETH...`);

  // The deposit function in SimplePaymaster.sol calls entryPoint.depositTo()
  // which requires the value to be sent with the transaction.
  const tx = await paymaster.connect(deployer).deposit({ value: fundingAmount });
  await tx.wait();
  console.log("Paymaster funded. Transaction hash:", tx.hash);

  // Verify the deposit in the EntryPoint for this paymaster
  const entryPointContract = await ethers.getContractAt("IEntryPoint", entryPointAddress, deployer);
  const paymasterDepositInEntryPoint = await entryPointContract.balanceOf(paymasterAddress);

  console.log(`Paymaster deposit in EntryPoint: ${ethers.formatEther(paymasterDepositInEntryPoint)} ETH`);

  if (paymasterDepositInEntryPoint >= fundingAmount) {
    console.log("Paymaster funding confirmed in EntryPoint.");
  } else {
    console.warn("Paymaster funding in EntryPoint is less than expected. Check transactions.");
  }

  return { paymasterAddress, paymasterDepositInEntryPoint };
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
