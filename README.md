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

Finally we need to install Garaga, make sure you have Python 3.10 in your system:

```sh
make install-garaga
```

Note that we need specific versions of Noir, Barretenberg, and Garaga to work well together. If you are experiencing any issues with code generation, proving, and verification — first of all ensure you have the correct package versions.

## Tutorial

This repo is organized in layers: each app iteration is a new git branch.  

Follow the steps and checkout the necessary branch:
1. [`master`](https://github.com/m-kus/scaffold-garaga/tree/master) — in-browser proof generation and stateless proof verification in devnet
2. [`1-app-logic`](https://github.com/m-kus/scaffold-garaga/tree/1-app-logic) — more involved Noir circuit logic
3. [`2-app-state`](https://github.com/m-kus/scaffold-garaga/tree/2-app-state) — extend onchain part with a storage for nullifiers
4. [`3-testnet`](https://github.com/m-kus/scaffold-garaga/tree/3-testnet) — deploy to public Starknet testnet and interact via wallet

## Run app

First of all we need to build our Noir circuit:

```sh
make build-circuit
```

Sample inputs are already provided in `Prover.toml`, execute to generate witness:

```sh
make exec-circuit
```

Generate verification key:

```sh
make gen-vk
```

Now we can generate the verifier contract in Cairo using Garaga:

```sh
make gen-verifier
```

Let's start our local development network in other terminal instance:

```sh
make devnet
```

Initialize the account we will be using for deployment:

```sh
make accounts-file
```

First we need to declare out contract ("upload" contract code):

```sh
make declare-contract
```

Now we can instantiate the contract class we obtained (you might need to update the command in Makefile):

```sh
make deploy-contract
```

Great! Now let's copy necessary artifacts and update contract address in the app code (change App.tsx):

```sh
make artifacts
```

Finally we can run the app:

```sh
make run-app
```

## Useful links

- Noir quickstart https://noir-lang.org/docs/getting_started/quick_start
- Garaga docs https://garaga.gitbook.io/garaga/deploy-your-snark-verifier-on-starknet/noir
- Starknet.js docs https://starknetjs.com/docs/guides/intro
- Starknet quickstart https://docs.starknet.io/quick-start/overview/
- Sncast 101 https://foundry-rs.github.io/starknet-foundry/starknet/101.html
- Cairo book https://book.cairo-lang.org/
