(define (domain robot-delivery-ma)
  (:requirements :strips :typing :negative-preconditions :multi-agent :unfactored-privacy)
  (:types robot location package)
  (:predicates
    (robot_at ?r - robot ?l - location)
    (package_at ?p - package ?l - location)
    (connected ?l1 ?l2 - location)
    (delivered ?p - package)
    (carrying ?r - robot ?p - package)
  )

  (:action move
    :agent ?r - robot
    :parameters (?from ?to - location)
    :precondition (and
      (robot_at ?r ?from)
      (connected ?from ?to)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
    )
  )

  (:action pickup
    :agent ?r - robot
    :parameters (?p - package ?l - location)
    :precondition (and
      (robot_at ?r ?l)
      (package_at ?p ?l)
    )
    :effect (and
      (not (package_at ?p ?l))
      (carrying ?r ?p)
    )
  )

  (:action deliver
    :agent ?r - robot
    :parameters (?p - package ?l - location)
    :precondition (and
      (robot_at ?r ?l)
      (carrying ?r ?p)
    )
    :effect (and
      (not (carrying ?r ?p))
      (package_at ?p ?l)
      (delivered ?p)
    )
  )
)