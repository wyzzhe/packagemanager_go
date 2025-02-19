# go-collide

[![godocs.io](https://godocs.io/git.sr.ht/~adnano/go-collide?status.svg)](https://godocs.io/git.sr.ht/~adnano/go-collide)

Package collide implements 2D collision detection in Go.

This package implements collision detection only. Resolving collisions is not implemented.

## Usage

	import "git.sr.ht/~adnano/go-collide"

## Examples

There are a few examples provided in the examples directory.
To run an example:

	go run examples/distance.go

## Contributing

Send patches and questions to [~adnano/go-collide-devel](https://lists.sr.ht/~adnano/go-collide-devel).

Subscribe to release announcements on [~adnano/go-collide-announce](https://lists.sr.ht/~adnano/go-collide-announce).

## Credits

This library is based on:

* Erin Catto's [Box2D](https://box2d.org)
* Randy Gaul's [ImpulseEngine](https://github.com/RandyGaul/ImpulseEngine)
* Chanhaeng Lee's [GJKPracticeBasedBox2D](https://github.com/lch32111/GJKPracticeBasedBox2D)

The following resources explain the various algorithms used in this library.

- [GJK: Collision detection algorithm](https://blog.winter.dev/2020/gjk-algorithm/)
- [Implementing GJK](https://caseymuratori.com/blog_0003)
- [Computing Distance using GJK](https://box2d.org/publications/#computing-distance-using-gjk-----gdc-2010)
- [Continuous Collision](https://box2d.org/publications/#continuous-collision-----gdc-2013)
