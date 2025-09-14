#pragma once

#include <memory>
#include <variant>
#include <optional>
#include <vector>
#include "state_machine/i_state.hpp"
#include "i_publisher.hpp"

namespace analog::speed_limiter
{

/**
 * @brief Identifiers for the different states in the speed limiter state machine.
 */
enum class StateId
{
  ESTOPPED,
  STOP,
  SLOW,
  FULL_SPEED,
  num_state_ids
};

/**
 * @brief Convert a StateId to a human-readable string.
 * @param state The StateId to convert
 * @return const char* The corresponding string representation
 */
const char * state_id_to_string(const StateId state);

/**
 * @brief Estop clear event
 */
struct EstopCleared {};

/**
 * @brief Estop triggered event
 */
struct EstopTriggered {};
/**
 * @brief Proximity data event
 */
struct ProximityData
{
  float distance;
};

/**
 * @brief Type-safe union of all possible events for the speed limiter state machine.
 */
using Events = std::variant<EstopCleared, EstopTriggered, ProximityData>;

/**
 * @brief Class containing common behavior for all states in the speed limiter state machine.
 */
class BaseState : public sm::IState<StateId, Events>
{
public:
  using PublisherPtr = std::shared_ptr<IPublisher<StateId>>;
  BaseState(StateId id, PublisherPtr publisher);

protected:
  /**
   * @see sm::IState::GetStateId
   */
  StateId GetStateId() override;

private:
  /**
   * @see sm::IState::Enter. Implementation publishes the current state.
   */
  void Enter() override;

  /**
   * @see sm::IState::Process. Implementation dispatches to specific Handle methods.
   */
  StateId Process(const Events & event) override;

  /**
   * @brief Upon receiving an EstopTriggered event, transition to ESTOPPED state.
   */
  StateId Handle(const EstopTriggered event);

  virtual StateId Handle(const EstopCleared event) = 0;
  virtual StateId Handle(const ProximityData event) = 0;

  StateId state_id_;
  PublisherPtr publisher_;
};

/**
 * @brief State entered when emergency stop is pressed.
 */
class EstoppedState : public BaseState
{
public:
  EstoppedState(PublisherPtr publisher);

private:
  /**
   * @brief Upon receiving an EstopCleared event, transition to STOP state.
   */
  StateId Handle(const EstopCleared event) override;

  /**
   * @brief Ignore ProximityData events while estopped.
   */
  StateId Handle(const ProximityData event) override;
};

/**
 * @brief Class for handling states when not estopped (e.g. FULL_SPEED, SLOW, STOP).
 */
class NotEstoppedState : public BaseState
{
public:
  /**
   * @brief Representation of upper limit for a speed region.
   */
  struct ProximityBoundary
  {
    StateId state; ///< State associated with region just below boundary
    float distance_mm; ///< Distance in mm representing the boundary
  };

  // All states except ESTOPPED have a boundary
  static constexpr std::size_t kNumProximityBoundaries =
    static_cast<std::size_t>(StateId::num_state_ids) - 1;

  using Boundaries = std::array<ProximityBoundary, kNumProximityBoundaries>;

  struct Params
  {
    Boundaries boundaries{{
      {StateId::STOP, 0.0},
      {StateId::SLOW, 0.0},
      {StateId::FULL_SPEED, 0.0}
    }};
    float hysteresis;
  };

  NotEstoppedState(StateId id, std::shared_ptr<Params> params, PublisherPtr publisher);

private:
  std::shared_ptr<Params> params_;

  /**
   * @brief Upon receiving an EstopCleared event, remain in the current state.
   */
  StateId Handle(const EstopCleared event) override;

  /**
   * @brief Upon receiving a ProximityData event, transition to the appropriate state.
   */
  StateId Handle(const ProximityData event) override;

  /**
   * @brief Helper function for checking parameters for validity.
   */
  static void CheckParams(const Params & params);
};

}
