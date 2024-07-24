#![allow(non_snake_case)]

use std::borrow::BorrowMut;

use dioxus::prelude::*;
use gloo_timers::future::TimeoutFuture;
use rapier2d::{
    dynamics::{
        CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
        RigidBody, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
    },
    geometry::{Collider, ColliderBuilder, ColliderSet, DefaultBroadPhase, NarrowPhase},
    na::Vector2,
    pipeline::{PhysicsPipeline, QueryPipeline},
};
use tracing::{info, Level};

fn main() {
    // Init logger
    dioxus_logger::init(Level::INFO).expect("failed to init logger");
    launch(App);
}

#[component]
fn App() -> Element {
    // Build cool things ✌️
    let mut rigid_body_set = use_signal(|| RigidBodySet::new());
    let mut collider_set = use_signal(|| ColliderSet::new());
    let mut ground_rg: Signal<Option<RigidBodyHandle>> = use_signal(|| None);
    let mut ball_rg: Signal<Option<Vec<RigidBodyHandle>>> = use_signal(|| None);
    // let mut is_drag = use_signal(|| false);
    // let is_drag = use_context_provider(|| is_drag);

    use_effect(move || {
        let rigid_body = RigidBodyBuilder::fixed()
            .translation(Vector2::new(0., -640.))
            .build();
        let collider = ColliderBuilder::cuboid(1000., 10.).build();
        let ground_body_handle = rigid_body_set.write().insert(rigid_body);
        collider_set.write().insert_with_parent(
            collider,
            ground_body_handle,
            &mut rigid_body_set.write(),
        );
        *ground_rg.write() = Some(ground_body_handle);
    });
    use_effect(move || {
        let count = 10;
        let mut temp = Vec::with_capacity(count);
        for i in 0..count {
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(Vector2::new((i * 62) as f32, -100.))
                .build();
            let collider = ColliderBuilder::ball(30.).restitution(1.).build();
            let ball_body_handle = rigid_body_set.write().insert(rigid_body);
            collider_set.write().insert_with_parent(
                collider,
                ball_body_handle,
                &mut rigid_body_set.write(),
            );
            temp.push(ball_body_handle);
        }
        *ball_rg.write() = Some(temp);
    });

        let gravity = use_signal(|| Vector2::new(0., -19.81));
        let integration_parameters = use_signal(|| IntegrationParameters::default());
        let mut island_manager = use_signal(||IslandManager::new());
        let mut broad_phase = use_signal(|| DefaultBroadPhase::new());
        let mut narrow_phase = use_signal(|| NarrowPhase::new());
        let mut impulse_joint_set = use_signal(|| ImpulseJointSet::new());
        let mut multibody_joint_set = use_signal(||MultibodyJointSet::new());
        let mut ccd_solver = use_signal(||CCDSolver::new());
        let mut query_pipeline = use_signal(||QueryPipeline::new());
        let physics_hooks = ();
        let event_handler = ();
        let mut physics_pipline = use_signal(||PhysicsPipeline::new());
    use_future(move || async move {
        loop {
            // if let Some(balls) = ball_rg(){
            //     if let Some(b) = balls.get(0){
            //         info!("{:?}", rigid_body_set()[*b].translation());
            //     }
            // }
            TimeoutFuture::new(10).await;
            // if !is_drag() {
                physics_pipline.write().step(
                    &gravity(),
                    &integration_parameters(),
                    &mut island_manager.write(),
                    &mut *broad_phase.write(),
                    &mut narrow_phase.write(),
                    &mut *rigid_body_set.write(),
                    &mut *collider_set.write(),
                    &mut impulse_joint_set.write(),
                    &mut multibody_joint_set.write(),
                    &mut ccd_solver.write(),
                    Some(&mut query_pipeline.write()),
                    &physics_hooks,
                    &event_handler,
                );
            // }
        }
    });

    if let (Some(balls), Some(ground)) = (ball_rg(), ground_rg()) {
        let ball_bodys = balls
            .into_iter()
            .map(|ball| {
                let mut is_drag = use_signal(|| false);
                let onmousedown = move |_|{
                    *is_drag.write() = true;                    
                    if let Some(body) = rigid_body_set.write().get_mut(ball){
                        body.set_body_type(rapier2d::dynamics::RigidBodyType::KinematicPositionBased, true);   
                    }
                    // info!("{}",is_drag());
                };
                let onmouseup = move |_|{
                    *is_drag.write() = false;
                    if let Some(body) = rigid_body_set.write().get_mut(ball){
                        body.set_body_type(rapier2d::dynamics::RigidBodyType::Dynamic, true);   
                    }
                    // info!("{}",is_drag());
                };
                let onmousemove = move |e: Event<MouseData>| {
                    if is_drag(){
                        let data = e.screen_coordinates();
                        let body = &rigid_body_set()[ball];
                        let col = &collider_set()[body.colliders()[0]];
                        let radius = col.shape().as_ball().unwrap().radius;
                        if let Some(body) = rigid_body_set.write().get_mut(ball){
                            body.set_translation(Vector2::new(data.x as f32 - radius * 1.5, (data.y as f32 - radius * 3.) * -1.), true);
                            info!("screen: {:?}",data);
                            info!("body: {:?}",body.translation());
                        }
                        // physics_pipline.write().step(
                        //     &gravity(),
                        //     &integration_parameters(),
                        //     &mut island_manager.write(),
                        //     &mut *broad_phase.write(),
                        //     &mut narrow_phase.write(),
                        //     &mut *rigid_body_set.write(),
                        //     &mut *collider_set.write(),
                        //     &mut impulse_joint_set.write(),
                        //     &mut multibody_joint_set.write(),
                        //     &mut ccd_solver.write(),
                        //     Some(&mut query_pipeline.write()),
                        //     &physics_hooks,
                        //     &event_handler,
                        // );
                    }
                };
                let body = &rigid_body_set()[ball];
                let col = &collider_set()[body.colliders()[0]];
                let radius = col.shape().as_ball().unwrap().radius;
                let x = body.translation().x;
                let y = body.translation().y - radius*-1.;
                rsx!{
                    div {
                        class: "unit ball rounded",
                        style: "--posx: {x}px;--posy: {y * -1.}px; --rot: {body.rotation().angle()}rad;--sizex:{radius*2.}px;--sizey:{radius*2.}px;",
                        onmousedown: onmousedown,
                        onmouseup: onmouseup,
                        onmousemove: onmousemove
                    }
                }
            });
        let ground_body = {
            let body = &rigid_body_set()[ground];
            let col = &collider_set()[body.colliders()[0]];
            let size = col.shape().as_cuboid().unwrap().half_extents;
            let x = body.translation().x;
            let y = body.translation().y - size.y * -1.;
            rsx! {
                div {
                    class: "unit ground",
                    style: "--posx: {x}px;--posy: {y * -1.}px; --rot: {body.rotation().angle()}rad;--sizex:{size.x*2.}px;--sizey:{size.y*2.}px;",
                }
            }
        };
        rsx! {
            link { rel: "stylesheet", href: "main.css" }
            for ball in ball_bodys{
                {ball}
            }
            {ground_body}
        }
    } else {
        rsx! {}
    }
}
