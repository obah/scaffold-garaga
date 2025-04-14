import { useState, useEffect } from 'react'
import './App.css'
import { ProofState, ProofStateData } from './types'
import { Noir } from "@noir-lang/noir_js";
import { bytecode, abi } from "./assets/circuit.json";
import initNoirC from "@noir-lang/noirc_abi";
import initACVM from "@noir-lang/acvm_js";
import acvm from "@noir-lang/acvm_js/web/acvm_js_bg.wasm?url";
import noirc from "@noir-lang/noirc_abi/web/noirc_abi_wasm_bg.wasm?url";

function App() {
  const [proofState, setProofState] = useState<ProofStateData>({
    state: ProofState.Initial
  });
  
  // Initialize WASM on component mount
  useEffect(() => {
    const initWasm = async () => {
      try {
        // This might have already been initialized in main.tsx,
        // but we're adding it here as a fallback
        if (typeof window !== 'undefined') {
          await Promise.all([initACVM(fetch(acvm)), initNoirC(fetch(noirc))]);
          console.log('WASM initialization in App component complete');
        }
      } catch (error) {
        console.error('Failed to initialize WASM in App component:', error);
      }
    };
    
    initWasm();
  }, []);

  const resetState = () => {
    setProofState({ state: ProofState.Initial });
  };

  const handleError = (error: unknown, errorAt: ProofState) => {
    console.error('Error:', error);
    setProofState({
      state: ProofState.Error,
      error: error instanceof Error ? error.message : 'An unknown error occurred',
      errorAt
    });
  };

  const startProcess = async () => {
    try {
      // Start the process
      setProofState({ state: ProofState.GeneratingWitness });
      
      // Mock input data
      const input = { x: 5, y: 10 };
      
      // Generate witness
      let noir = new Noir({ bytecode, abi: abi as any });
      let execResult = await noir.execute(input);
      console.log(execResult);
      
      // Generate proof
      setProofState({ state: ProofState.GeneratingProof });
      
      // Prepare calldata
      setProofState({ state: ProofState.PreparingCalldata });
      
      // Connect wallet
      setProofState({ state: ProofState.ConnectingWallet });
      
      // Send transaction
      setProofState({ state: ProofState.SendingTransaction });
      
      // Check verification
      const verified = true;
      
      if (verified) {
        setProofState({ state: ProofState.ProofVerified });
      } else {
        throw new Error('Proof verification failed');
      }
    } catch (error) {
      handleError(error, proofState.state);
    }
  };

  const renderStateIndicator = (state: ProofState, current: ProofState, errorAt?: ProofState) => {
    let status = 'pending';
    
    if (current === ProofState.Error && errorAt === state) {
      status = 'error';
    } else if (current === state) {
      status = 'active';
    } else if (current === ProofState.Error && getStateIndex(errorAt || ProofState.Initial) < getStateIndex(state)) {
      status = 'pending';
    } else if (getStateIndex(current) > getStateIndex(state)) {
      status = 'completed';
    }
    
    return (
      <div className={`state-indicator ${status}`}>
        <div className="state-dot"></div>
        <div className="state-label">{state}</div>
      </div>
    );
  };

  const getStateIndex = (state: ProofState): number => {
    const states = [
      ProofState.Initial,
      ProofState.GeneratingWitness,
      ProofState.GeneratingProof,
      ProofState.PreparingCalldata,
      ProofState.ConnectingWallet,
      ProofState.SendingTransaction,
      ProofState.ProofVerified
    ];
    
    return states.indexOf(state);
  };

  return (
    <div className="container">
      <h1>Noir Proof Generation & Starknet Verification</h1>
      
      <div className="state-machine">
        {renderStateIndicator(ProofState.GeneratingWitness, proofState.state, proofState.errorAt)}
        {renderStateIndicator(ProofState.GeneratingProof, proofState.state, proofState.errorAt)}
        {renderStateIndicator(ProofState.PreparingCalldata, proofState.state, proofState.errorAt)}
        {renderStateIndicator(ProofState.ConnectingWallet, proofState.state, proofState.errorAt)}
        {renderStateIndicator(ProofState.SendingTransaction, proofState.state, proofState.errorAt)}
      </div>
      
      {proofState.error && (
        <div className="error-message">
          Error at stage '{proofState.errorAt}': {proofState.error}
        </div>
      )}
      
      <div className="controls">
        {proofState.state === ProofState.Initial && (
          <button className="primary-button" onClick={startProcess}>Start</button>
        )}
        
        {(proofState.state === ProofState.Error || proofState.state === ProofState.ProofVerified) && (
          <button className="reset-button" onClick={resetState}>Reset</button>
        )}
      </div>
    </div>
  )
}

export default App
