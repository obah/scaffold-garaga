install-noir:
	curl -L https://raw.githubusercontent.com/noir-lang/noirup/refs/heads/main/install | bash
	noirup --version 1.0.0-beta.2

install-barretenberg:
	curl -L https://raw.githubusercontent.com/AztecProtocol/aztec-packages/refs/heads/master/barretenberg/bbup/install | bash
	bbup --version 0.82.2

install-starknet:
	curl --proto '=https' --tlsv1.2 -sSf https://sh.starkup.dev | sh

install-devnet:
	asdf plugin add starknet-devnet
	asdf install starknet-devnet 0.3.0

devnet:
	starknet-devnet --accounts=2 --seed=0

build-circuit:
	cd circuit && nargo build

exec-circuit:
	cd circuit && nargo execute witness

prove-circuit:
	bb prove --scheme ultra_honk --oracle_hash keccak -b ./circuit/target/circuit.json -w ./circuit/target/witness.gz -o ./circuit/target

gen-vk:
	bb write_vk --scheme ultra_honk --oracle_hash keccak -b ./circuit/target/circuit.json -o ./circuit/target

gen-contract:
	cd contracts && garaga gen --system ultra_keccak_honk --vk ../circuit/target/vk --project-name verifier

build-contract:
	cd contracts && scarb build

artifacts:
	cp ./circuit/target/circuit.json ./app/src/circuit/circuit.json
	cp ./circuit/target/vk ./app/public/vk.bin
