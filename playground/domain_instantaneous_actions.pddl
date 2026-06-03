(define (domain logistics-simple)
  (:requirements :strips :typing :fluents :disjunctive-preconditions :negative-preconditions)
  
  (:types
    location vehicle package
  )


  (:constants
    truck2 - vehicle
    truck3 - vehicle
  )

  (:predicates
    (at-vehicle ?v - vehicle ?l - location)
    (at-package ?p - package ?l - location)
    (in-vehicle ?p - package ?v - vehicle)
    (connected ?l1 - location ?l2 - location)
  )

  (:functions
    (max-load ?v - vehicle)
  )

  
  ;; Acción: mover un vehículo de una ubicación a otra
  (:action drive
    :parameters (?v - vehicle ?from - location ?to - location)
    :precondition (and
      (at-vehicle ?v ?from)
      (connected ?from ?to)
    )
    :effect (and
      (not (at-vehicle ?v ?from))
      (at-vehicle ?v ?to)
    )
  )
  
  ;; Acción: cargar un paquete en un vehículo
  (:action load
    :parameters (?p - package ?v - vehicle ?l - location)
    :precondition (and
      (at-package ?p ?l)
      (at-vehicle ?v ?l)
    )
    :effect (and
      (not (at-package ?p ?l))
      (in-vehicle ?p ?v)
    )
  )
  
  ;; Acción: descargar un paquete de un vehículo
  (:action unload
    :parameters (?p - package ?v - vehicle ?l - location)
    :precondition (and
      (in-vehicle ?p ?v)
      (at-vehicle ?v ?l)
    )
    :effect (and
      (not (in-vehicle ?p ?v))
      (at-package ?p ?l)
    )
  )
)