// Dagger CI module for aspeed-rs.
//
// Usage (from the aspeed-rs repo root):
//
//	dagger call check --aspeed-data ../aspeed-data    # compile all chip targets
//	dagger call test                                  # host-side unit tests
//	dagger call ci --aspeed-data ../aspeed-data       # full pipeline
package main

import (
	"context"
	"fmt"
	"strings"

	"dagger/aspeed-rs/internal/dagger"
)

const rustChannel = "nightly-2026-04-01"

// chipTargets lists every (feature, triple) that embassy-aspeed must compile for.
var chipTargets = []struct{ feature, triple string }{
	{"ast2600-ssp", "thumbv7m-none-eabi"},
	{"ast1060", "thumbv7em-none-eabihf"},
	{"ast2700-bootmcu", "riscv32imc-unknown-none-elf"},
}

type AspeedRs struct{}

// rustContainer returns a Rust nightly container with all three embedded targets
// and the aspeed-data sibling mounted at the path the Cargo.toml expects.
func (m *AspeedRs) rustContainer(src *dagger.Directory, aspeedData *dagger.Directory) *dagger.Container {
	cargoCache := dag.CacheVolume("cargo-registry")
	buildCache := dag.CacheVolume("cargo-build-aspeed-rs")

	return dag.Container().
		From("rust:1-slim").
		WithExec([]string{
			"rustup", "toolchain", "install", rustChannel,
			"--profile", "minimal",
			"--target", "thumbv7m-none-eabi",
			"--target", "thumbv7em-none-eabihf",
			"--target", "riscv32imc-unknown-none-elf",
			"--component", "rustfmt,clippy",
			"--no-self-update",
		}).
		WithExec([]string{"rustup", "default", rustChannel}).
		WithMountedCache("/usr/local/cargo/registry", cargoCache).
		WithMountedCache("/build/target", buildCache).
		// aspeed-pac is referenced via path = "../../aspeed-data/aspeed-pac"
		WithDirectory("/build/aspeed-data", aspeedData).
		WithDirectory("/build/aspeed-rs", src).
		WithWorkdir("/build/aspeed-rs")
}

// hostContainer returns a Rust nightly container for host-target tests (x86_64).
// aspeed-pac is a path dep so aspeed-data must still be mounted.
func (m *AspeedRs) hostContainer(src *dagger.Directory, aspeedData *dagger.Directory) *dagger.Container {
	cargoCache := dag.CacheVolume("cargo-registry")
	return dag.Container().
		From("rust:1-slim").
		WithExec([]string{
			"rustup", "toolchain", "install", rustChannel,
			"--profile", "minimal",
			"--component", "rustfmt,clippy",
			"--no-self-update",
		}).
		WithExec([]string{"rustup", "default", rustChannel}).
		WithMountedCache("/usr/local/cargo/registry", cargoCache).
		WithDirectory("/build/aspeed-data", aspeedData).
		WithDirectory("/build/aspeed-rs", src).
		WithWorkdir("/build/aspeed-rs")
}

// Check compiles embassy-aspeed for all three chip targets.
// Pass the aspeed-data sibling dir: --aspeed-data ../aspeed-data
func (m *AspeedRs) Check(
	ctx context.Context,
	// +defaultPath="."
	src *dagger.Directory,
	aspeedData *dagger.Directory,
) error {
	ctr := m.rustContainer(src, aspeedData)
	for _, t := range chipTargets {
		_, err := ctr.
			WithExec([]string{
				"cargo", "check", "-p", "embassy-aspeed",
				"--features", t.feature,
				"--target", t.triple,
			}).
			Sync(ctx)
		if err != nil {
			return fmt.Errorf("check feature=%s target=%s: %w", t.feature, t.triple, err)
		}
	}
	return nil
}

// Clippy runs Clippy on the ast2600-ssp target.
// Pass the aspeed-data sibling dir: --aspeed-data ../aspeed-data
func (m *AspeedRs) Clippy(
	ctx context.Context,
	// +defaultPath="."
	src *dagger.Directory,
	aspeedData *dagger.Directory,
) error {
	_, err := m.rustContainer(src, aspeedData).
		WithExec([]string{
			"cargo", "clippy", "-p", "embassy-aspeed",
			"--features", "ast2600-ssp",
			"--target", "thumbv7m-none-eabi",
			"--", "-D", "warnings",
		}).
		Sync(ctx)
	return err
}

// Test runs host-side unit tests (no chip feature, compiles for x86_64).
// Pass the aspeed-data sibling dir: --aspeed-data ../aspeed-data
func (m *AspeedRs) Test(
	ctx context.Context,
	// +defaultPath="."
	src *dagger.Directory,
	aspeedData *dagger.Directory,
) error {
	_, err := m.hostContainer(src, aspeedData).
		WithExec([]string{
			"cargo", "test", "-p", "embassy-aspeed",
			"--lib",
			"--target", "x86_64-unknown-linux-gnu",
		}).
		Sync(ctx)
	return err
}

// FmtCheck verifies Rust formatting.
// Pass the aspeed-data sibling dir: --aspeed-data ../aspeed-data
func (m *AspeedRs) FmtCheck(
	ctx context.Context,
	// +defaultPath="."
	src *dagger.Directory,
	aspeedData *dagger.Directory,
) error {
	_, err := m.hostContainer(src, aspeedData).
		WithExec([]string{"cargo", "fmt", "--all", "--", "--check"}).
		Sync(ctx)
	return err
}

// Ci runs the full pipeline: Check + Test + FmtCheck.
// Pass the aspeed-data sibling dir: --aspeed-data ../aspeed-data
func (m *AspeedRs) Ci(
	ctx context.Context,
	// +defaultPath="."
	src *dagger.Directory,
	aspeedData *dagger.Directory,
) (string, error) {
	steps := []string{}

	if err := m.Check(ctx, src, aspeedData); err != nil {
		return "", fmt.Errorf("check: %w", err)
	}
	steps = append(steps, "check: ok")

	if err := m.Test(ctx, src, aspeedData); err != nil {
		return "", fmt.Errorf("test: %w", err)
	}
	steps = append(steps, "test: ok")

	if err := m.FmtCheck(ctx, src, aspeedData); err != nil {
		return "", fmt.Errorf("fmt-check: %w", err)
	}
	steps = append(steps, "fmt-check: ok")

	return strings.Join(steps, "\n"), nil
}
