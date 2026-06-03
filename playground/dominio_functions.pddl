(define (domain test_domain)
  (:requirements :strips :typing :fluents)
  
  (:types
    location
  )
  
  (:predicates
    (at ?l - location)
  )
  
  (:functions
    (temperature ?l - location)
    (pressure ?l - location)
    (humidity ?l - location)
  )
  
  (:action test
    :parameters (?l - location)
    :effect (and
      (increase (temperature ?l) 5.0)
      (decrease (pressure ?l) 2.0)
      (assign (humidity ?l) 50.0)
    )
  )
)