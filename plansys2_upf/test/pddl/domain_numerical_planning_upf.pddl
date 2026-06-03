(define (domain robot-logistics)
  (:requirements :strips :typing :numeric-fluents)
  (:types robot location package)
  (:predicates
    (robot_at ?r - robot ?l - location)
    (package_at ?p - package ?l - location)
    (carrying ?r - robot ?p - package)
    (connected ?l1 ?l2 - location)
    (charging_point_at ?l - location)
  )
  (:functions
    (battery ?r - robot)
    (move_cost ?l1 ?l2 - location)
    (packages_carried ?r - robot)
    (total_deliveries)
  )
  (:action move
    :parameters (?r - robot ?from ?to - location)
    :precondition (and
      (robot_at ?r ?from)
      (connected ?from ?to)
      (>= (battery ?r) (move_cost ?from ?to))
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
      (decrease (battery ?r) (move_cost ?from ?to))
    )
  )
  (:action pickup
    :parameters (?r - robot ?p - package ?l - location)
    :precondition (and
      (robot_at ?r ?l)
      (package_at ?p ?l)
      (< (packages_carried ?r) 3)
    )
    :effect (and
      (not (package_at ?p ?l))
      (carrying ?r ?p)
      (increase (packages_carried ?r) 1)
    )
  )
  (:action deliver
    :parameters (?r - robot ?p - package ?l - location)
    :precondition (and
      (robot_at ?r ?l)
      (carrying ?r ?p)
    )
    :effect (and
      (not (carrying ?r ?p))
      (package_at ?p ?l)
      (decrease (packages_carried ?r) 1)
      (increase (total_deliveries) 1)
    )
  )
  (:action charge
    :parameters (?r - robot ?l - location)
    :precondition (and
      (robot_at ?r ?l)
      (charging_point_at ?l)
    )
    :effect (increase (battery ?r) 100)
  )
)