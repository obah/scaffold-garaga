# Scaffold Garaga app

This is a Noir+Garaga+Starknet starter with in-browser proving and a step-by-step guide how to:
- Generate and deploy UltraHonk proof verifier contract to Starknet devnet
- Add state to your privacy preserving app
- Add wallet connection and deploy to public testnet

## Install

Ensure you have node.js >= 20 installed.  

Bun is used for package management, install it with:
```sh
make install-bun
```

For compiling Noir circuits and generating proofs we need specific versions of Aztec packages:
```sh
make install-noir
make install-barretenberg
```

Starknet toolkit comes in a single bundle via asdf (the following command will install it if you don't have it):
```sh
make install-starknet
```

We also need to install a tool for spawning local Starknet chain:
```sh
make install-devnet
```

## Guides

This repo is organized in layers: every next app iteration is a new git branch.  

Follow the steps and checkout the necessary branch:
1. [`master`](https://github.com/m-kus/scaffold-garaga/tree/master) — in-browser proof generation and stateless proof verification in devnet
2. [`1-deploy-verifier`](https://github.com/m-kus/scaffold-garaga/tree/1-deploy-verifier) — more involved Noir logic and contract deployment guide
3. [`2-app-state`](https://github.com/m-kus/scaffold-garaga/tree/2-app-state) — extend onchain part with a storage for nullifiers (application state)
4. [`3-connect-wallet`](https://github.com/m-kus/scaffold-garaga/tree/3-connect-wallet) — deploy to public Starknet testnet and interact via wallet
