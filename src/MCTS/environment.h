#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <tuple>

using std::tuple;

/** This is an abstraction of an environment for MCTS. The environment essentially only needs to simulate (=transition
 *  forward the state for given actions), and for each transition return the observation and reward. Additionally, it
 *  needs to provide the set of feasible decisions for the current state. For the MCTS solver, states, actions, and
 *  rewards are fully abstract entities -- they can only be referred to via 'handles'. */
struct MCTS_Environment {
    /** A generic State-Action-Observation object as abstraction of a real state, action, or observation. The environment can,
     *  via dynamic casting, get the semantics back. The MCTS solver should not use any other properties than equality. */
    struct SAO {
      virtual ~SAO(){}
      virtual bool operator==(const SAO & other) const { return true; } ///< why true? Because a NIL object (e.g. non-observation) is always equal to NIL
      virtual bool operator!=(const SAO & other) const { return !(*this==other); }
      virtual void write(std::ostream& os) const { os <<"NIL_SAO"; }
        virtual size_t get_hash() const {
            std::cout << "NOT IMPLEMENTED YET -- HARD EXIT!!!" << std::endl;
            exit(-1);
        }
    };
    typedef std::shared_ptr<const SAO> Handle;

    MCTS_Environment() = default;
  virtual ~MCTS_Environment(){}

    /// Perform the action; return the resulting observation and reward
    virtual std::pair<Handle, double> transition(const Handle& action) = 0;

    /// Perform a random action
    virtual std::pair<Handle, double> transition_randomly(){
      std::vector<Handle> actions = get_actions();
      return transition(actions[rand()%actions.size()]);
    }

    /// Get the available actions in the current state
    virtual const std::vector<Handle> get_actions() = 0;

    /// Get the current state
    virtual const Handle get_state() = 0;

    /// Return whether the current state is a terminal state
    virtual bool is_terminal_state() const = 0;

    /// Makes the current state the future start state set by reset_state().
    virtual void make_current_state_default() = 0;

    /// Reset the environment's state to the start state
    virtual void reset_state() = 0;

    /// static information on the environment
    enum InfoTag{ getGamma, hasTerminal, isDeterministic, hasMaxReward, getMaxReward, hasMinReward, getMinReward, isMarkov, writeState };
    virtual bool get_info(InfoTag tag) const = 0;
    virtual double get_info_value(InfoTag tag) const = 0;
};


inline std::ostream& operator<<(std::ostream& os, const MCTS_Environment::SAO& x){ x.write(os); return os; }