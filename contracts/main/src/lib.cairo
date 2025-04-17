#[starknet::interface]
trait IMainContract<TContractState> {
    fn add_solution(ref self: TContractState, full_proof_with_hints: Span<felt252>);
}

#[starknet::contract]
mod MainContract {
    use starknet::storage::{
        StoragePointerReadAccess, StoragePointerWriteAccess, StoragePathEntry, Map,
    };
    use starknet::{syscalls, SyscallResultTrait};

<<<<<<< HEAD
    // TODO: use class hash from the result of the `make declare-verifier` step
    const VERIFIER_CLASSHASH: felt252 = 0x00bb20462f9741231dca2052a0d4b15d1c7c91b3ba0df91cb264e4f9fd5e80cc;
=======
    const VERIFIER_CLASSHASH: felt252 = 0x065810e6858268556a5e6ac4d2e83cf35893c044db9ebfa308ed5e5d4cb58a61;
>>>>>>> 3ff862d (Add new contract with state)

    #[storage]
    struct Storage {
        // Don't do that for a real use case, use merkle tree instead
        nullifiers: Map<u256, bool>,
        public_key: u256,
    }

    #[constructor]
    fn constructor(ref self: ContractState, public_key: u256) {
        self.public_key.write(public_key);
    }

    #[abi(embed_v0)]
    impl IMainContractImpl of super::IMainContract<ContractState> {
        fn add_solution(ref self: ContractState, full_proof_with_hints: Span<felt252>) {
            let mut res = syscalls::library_call_syscall(
                VERIFIER_CLASSHASH.try_into().unwrap(),
                selector!("verify_ultra_keccak_honk_proof"),
                full_proof_with_hints
            )
                .unwrap_syscall();
            let public_inputs = Serde::<Option<Span<u256>>>::deserialize(ref res).unwrap().expect('Proof is invalid');

            let public_key = *public_inputs[0];
            let nullifier = *public_inputs[1];

            assert(self.public_key.read() == public_key, 'Public key does not match');
            assert(self.nullifiers.entry(nullifier).read() == false, 'Nullifier already used');

            self.nullifiers.entry(nullifier).write(true);
        }
    }
}
